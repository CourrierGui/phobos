/* -*- mode: c; c-basic-offset: 4; indent-tabs-mode: nil; -*-
 * vim:expandtab:shiftwidth=4:tabstop=4:
 */
/*
 *  All rights reserved (c) 2014-2025 CEA/DAM.
 *
 *  This file is part of Phobos.
 *
 *  Phobos is free software: you can redistribute it and/or modify it under
 *  the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  Phobos is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Phobos. If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * \brief  LRS Grouped Write I/O Scheduler: group write request per tag and/or
 * groupings.
 */
#include "lrs_sched.h"
#include "lrs_utils.h"
#include "pho_common.h"
#include "pho_types.h"
#include "schedulers.h"

// TODO add library to list criteria?
/* Principle of the algorithm:
 *
 * The goal is to group write requests per tag and RAID parameters (e.g.
 * n_media). The assumption is that upper layers will be smart enough to push
 * writes to the LRS so that by performing them in order the devices will be
 * used most of the time. For instance, one device won't stay idle waiting for
 * a second device to be available for a RAID1/repl_count=2 write.
 *
 * We also group writes per grouping on the tapes. This will increase data
 * locality to reduce the read latency. Less tape seeks will be necessary to
 * read all the data of a given grouping.
 *
 * To achieve this, requests are put in queues (struct gw_queue) that are stored
 * in an hashtable. They are indexed by layout parameters and tags. Each
 * gw_queue contains a list of queues. One queue per grouping. When new requests
 * are pushed, the appropriate queue is retrieved from the hash table using
 * the request's parameters. The request is then inserted in the appropriate
 * grouping's queue.
 *
 * One assumption made by this algorithm is that all the media of a given
 * write request all use the same tags. The protocol seems to support
 * different tags per medium. This is not currently the case in Phobos.
 *
 * Note that this algorithm is not really concerned with the state of the media.
 * This is managed by the upper layers. If a medium becomes failed due to an I/O
 * error for instance, the LRS will unload it if it can and then ask for a new
 * medium. Either by calling .retry if the error occures before the response
 * is sent back to the client or by simply removing the request and trying the
 * next one. We also don't change the streams' devices in this case. If the
 * device itself becomes failed, .remove_device will be called and the device
 * will be removed from the stream it belongs to. A new device will be selected
 * during the next allocation attempt.
 *
 * XXX This behavior can be very suboptimal when a device failure prevents a
 * stream from allocating new requests. It will be stuck until a device becomes
 * available. A better approach would be to remove the stream from the allocated
 * streams on .remove_device to let other streams be scheduled instead. This
 * breaks the current assumption that we don't reorder queues and groupings.
 * But this could be acceptable in the face of a device failure.
 */

#define walloc_grouping(reqc) \
    (reqc)->req->walloc->grouping

/* Simple wrapper macro to avoid mistakes when choosing from which side to
 * push/pop
 */
#define queue_push(queue, value) \
    g_queue_push_tail((queue), (value))

#define queue_pop(queue) \
    g_queue_pop_head(queue)

#define queue_peek(queue) \
    g_queue_peek_head(queue)

#define queue_foreach(iter, queue) \
    glist_foreach(iter, (queue)->head)

struct gw_request {
    struct req_container *reqc;
    /** Pointer to the queue containing this request */
    struct gw_queue *queue;
};

struct gw_grouping {
    /** Name of the grouping used as the key in gw_queue::grouping_index */
    char *name;
    /** List of struct gw_request for this grouping */
    GQueue *requests;
};

/**
 * Instances of this structure represent a list of devices that
 * are used by a queue to push requests. All the streams of a given
 * queue must have the same number of devices gw_queue::n_media.
 *
 * The grouping (i.e. a list of requests of the same grouping)
 * currently written is associted to a stream. This grouping
 * is switched with a new one when the list of requests is empty.
 */
struct gw_data_stream {
    /** List of size gw_queue::n_media. Devices used to allocate requests
     * for this stream
     */
    struct lrs_dev **devices;
    /** The grouping whose requests are currently beeing pushed to the
     * devices of \p devices. The grouping can be shared amongst different
     * streams.
     */
    struct gw_grouping *grouping;
    /** Reference back to the queue owning this stream */
    struct gw_queue *queue;
};

/**
 * All the requests in an instance of this structure must have the same tags.
 * Requests are grouped and scheduled by number of drives required (e.g. RAID
 * params).
 *
 * The queues are created when requests with new tags+RAID params are received
 * and free'd when we need to switch queue. We can only switch queue when the
 * list is empty.
 */
struct gw_queue {
  /** Type of layout (e.g. RAID1, RAID4).
   * For now, this will always be 0 but can be used in the future to
   * differentiate between RAID1 repl_count=3 and RAID4.
   */
  int layout_type;
  /** Number of media per request */
  gint64 n_media;
  /** comma seperated list of tags */
  const char *tags;

  /** List of struct gw_grouping in the order of each groupings' first request
   * received by the scheduler. This is used to write groupings in order.
   */
  GQueue *groupings;
  /** map grouping name to struct gw_grouping for fast lookup.
   * NULL is supported as a key to store requests without grouping.
   */
  GHashTable *grouping_index;

  /** List of struct gw_data_stream. Each stream represents a serialized queue
   * of requests meaning that requests from a single stream will be pushed one
   * by one to the stream's devices. To send requests from a single grouping in
   * parallel, multiple streams must be created. A queue with multiple groupings
   * can also be split into multiple streams by allocating different groupings
   * to different streams.
   */
  GList *streams;

  /**
   * Index of the current medium to allocate in .get_device_medium_pair()
   * Used to detect whether the caller is retrying to allocate a device
   * that was not ready for use due to concurrency effects for instance.
   *
   * First initialized to n_media by .peek_request() since the index is an
   * unsigned integer, we have to use a positive value to detect the first index
   * of 0.
   */
  size_t alloc_index;
};

/** We need to keep track of how many concurrent streams are run for a single
 * grouping. A gives grouping could be found in different queues if the requests
 * target different tags for instance.
 *
 * TODO max_streams can be read from configuration to specify the maximum
 * desired parallelism per grouping.
 */
struct gw_grouping_config {
    /** grouping's name */
    char *grouping;
    /** Maximum number of data streams for this grouping */
    size_t max_streams;
    /** Current number of streams used for this grouping */
    size_t current_streams;
};

/** Global state of the scheduler */
struct gw_state {
    /** key = value = struct gw_queue.
     *
     * gw_queue is hashed based on the layout type, number of media per request
     * and tag list. This information is contained in the struct gw_queue. There
     * is no external key. Therefore, gw_queue is the key and the value.
     */
    GHashTable *queues;

    /**
     * List of queues in \p queues added in the order they are created
     * to match the order in which they are sent to the LRS.
     */
    GList *ordered_queues;

    /** List of queues allocated to devices. Only the requests from these queues
     * can be allocated to devices. They are removed only when empty.
     */
    GList *allocated;

    /** Request currently being allocated (peek -> get_device_medium_pair ->
     * remove/request cycle)
     */
    struct gw_request *current_request;

    /** Stream that was allocated during peek_request */
    struct gw_data_stream *current_stream;

    /** List of devices eligible for scheduling a new stream */
    GPtrArray *free_devices;

    /** This list contains the devices that have been allocated by
     * .get_device_medium_pair for the last request of a grouping.
     * The goal is to keep track of all the I/O running for a given grouping.
     * Since the stream is removed by .remove_request on the last request of the
     * grouping, we have to keep track of those devices separately.
     */
    GPtrArray *busy_devices;

    /** Hash table to quickly find a stream given a device. */
    GHashTable *device_to_stream;

    /** key = grouping_name, value = struct gw_grouping_config
     * The config is global to all groupings. If the same grouping
     * is pushed with different tags or RAID parameters, we need to
     * ensure that the maximum number of groupings written in parallel
     * does not exceed the config.
     */
    GHashTable *data_streams_config;

    /** If max streams per grouping is not specified, use this
     * default value instead.
     */
    size_t default_max_streams;

    /** Minimum size a queue must have before we can consider splitting it into
     * multiple streams. This avoids breaking up a list into too quickly which
     * would favor groupings that arrive first too much. Also, we don't want to
     * run requests in parallel if there are only a handful of them.
     */
    size_t min_reqs_before_split;
};

static struct gw_grouping *gw_grouping_new(const char *name)
{
    struct gw_grouping *grouping;

    grouping = xmalloc(sizeof(*grouping));
    grouping->name = xstrdup_safe(name);
    grouping->requests = g_queue_new();

    return grouping;
}

static void gw_grouping_free(struct gw_grouping *grouping)
{
    g_queue_free(grouping->requests);
    free(grouping->name);
    free(grouping);
}

static guint glib_wqueue_hash(gconstpointer v)
{
    const struct gw_queue *queue = v;

    return g_str_hash(queue->tags) ^ g_int64_hash(&queue->n_media) ^
        g_int_hash(&queue->layout_type);
}

static gboolean glib_wqueue_equal(gconstpointer _lhs, gconstpointer _rhs)
{
    const struct gw_queue *lhs = _lhs;
    const struct gw_queue *rhs = _rhs;

    return (lhs->layout_type == rhs->layout_type) &&
        (lhs->n_media == rhs->n_media) &&
        !strcmp(lhs->tags, rhs->tags);
}

static int gw_init(struct io_scheduler *io_sched)
{
    struct gw_state *state;

    state = xcalloc(1, sizeof(*state));
    /* explicit initialization even though it is already NULL just to show
     * that we have empty GLists
     */
    state->allocated = NULL;
    state->ordered_queues = NULL;
    state->queues = g_hash_table_new(glib_wqueue_hash, glib_wqueue_equal);

    state->device_to_stream = g_hash_table_new(g_direct_hash, g_direct_equal);
    state->free_devices = g_ptr_array_new();
    state->busy_devices = g_ptr_array_new();

    /* TODO load these from config */
    state->min_reqs_before_split = 2;
    state->default_max_streams = 1;
    state->data_streams_config = g_hash_table_new(pho_glib_nullable_str_hash,
                                                  pho_glib_nullable_str_equal);

    io_sched->private_data = state;

    return 0;
}

static void gw_fini(struct io_scheduler *io_sched)
{
    struct gw_state *state = io_sched->private_data;

    g_hash_table_destroy(state->data_streams_config);
    g_ptr_array_free(state->busy_devices, TRUE);
    g_ptr_array_free(state->free_devices, TRUE);
    g_hash_table_destroy(state->device_to_stream);
    g_hash_table_destroy(state->queues);
    g_list_free(state->ordered_queues);
    g_list_free(state->allocated);
    free(state);
}

static struct gw_data_stream *gw_data_stream_new(struct gw_queue *queue)
{
    struct gw_data_stream *stream;

    stream = xmalloc(sizeof(*stream));
    stream->queue = queue;
    stream->devices = xcalloc(queue->n_media, sizeof(*stream->devices));

    return stream;
}

static void gw_data_stream_free(struct gw_data_stream *stream)
{
    free(stream->devices);
    free(stream);
}

static void gw_data_stream_set_device(struct gw_state *state,
                                      struct gw_data_stream *stream,
                                      struct lrs_dev *dev,
                                      size_t index)
{
    if (stream->devices[index])
        g_hash_table_remove(state->device_to_stream, dev);

    stream->devices[index] = dev;
    g_hash_table_insert(state->device_to_stream, dev, stream);
}

static struct gw_data_stream *
gw_data_stream_remove_device(struct gw_state *state,
                             struct lrs_dev *device,
                             size_t index)
{
    struct gw_data_stream *stream;

    stream = g_hash_table_lookup(state->device_to_stream, device);
    if (!stream)
        /* In .retry, the stream might have been removed by .remove_request if
         * it was the last request of the stream.
         */
        return NULL;

    g_hash_table_remove(state->device_to_stream, device);
    stream->devices[index] = NULL;

    return stream;
}

static void gw_data_stream_finished(struct gw_state *state,
                                    struct gw_data_stream *stream)
{
    guint64 i;

    /* devices will be put back in the free list by handle_finished_io */
    for (i = 0; i < stream->queue->n_media; i++) {
        g_ptr_array_add(state->busy_devices, stream->devices[i]);
        gw_data_stream_remove_device(state, stream->devices[i], i);
    }

    gw_data_stream_free(stream);
}

static gint glib_strcmp(gconstpointer a, gconstpointer b)
{
    return strcmp((const char *)a, (const char *)b);
}

static GList *sorted_tags(char **tags, size_t n_tags)
{
    GList *res = NULL;
    size_t i;

    for (i = 0; i < n_tags; i++)
        res = g_list_append(res, tags[i]);

    return g_list_sort(res, glib_strcmp);
}

static char *tag2csv(pho_req_write_t *walloc)
{
    GList *tags = NULL;
    char *tag_list;
    size_t len = 0;
    char *iter;

    assert(walloc->n_media > 0);

    /* sort tags so that tags "foo,bar" and "bar,foo" are the same */
    tags = sorted_tags(walloc->media[0]->tags,
                       walloc->media[0]->n_tags);

    glist_foreach(tag, tags)
        len += strlen(tag->data) + 1; // + 1 for the ','

    if (g_list_length(tags) > 0)
        len--; // remove last +1 for ','

    tag_list = xmalloc(len + 1); // +1 for '\0'
    iter = tag_list;

    glist_foreach(tag, tags) {
        size_t len = strlen(tag->data);

        memcpy(iter, tag->data, len);
        iter += len;
        *iter++ = ',';
    }
    if (g_list_length(tags) > 0)
        iter--; // move iter back to overwrite last ','
    *iter = '\0';

    pho_debug("build tag_list '%s' for '%p'", tag_list, walloc);

    g_list_free(tags);
    return tag_list;
}

static struct gw_queue *gw_find_queue(struct gw_state *state,
                                      struct req_container *reqc)
{
    struct gw_queue tmp = {
        .n_media = reqc->req->walloc->n_media,
        .layout_type = 0,
        .tags = tag2csv(reqc->req->walloc),
    };
    struct gw_queue *queue;

    queue = g_hash_table_lookup(state->queues, &tmp);
    free((void *)tmp.tags);

    return queue;
}

static struct gw_queue *gw_queue_create(struct gw_state *state,
                                        struct req_container *reqc)
{
    struct gw_queue *queue = xcalloc(1, sizeof(*queue));

    queue->tags = tag2csv(reqc->req->walloc);
    queue->n_media = reqc->req->walloc->n_media;
    queue->groupings = g_queue_new();
    queue->grouping_index = g_hash_table_new(g_str_hash, g_str_equal);
    queue->layout_type = 0;
    queue->alloc_index = queue->n_media;

    g_hash_table_insert(state->queues, queue, queue);
    state->ordered_queues = g_list_append(state->ordered_queues, queue);

    return queue;
}

static void gw_queue_destroy(struct gw_state *state, struct gw_queue *queue)
{
    /* The queue might not be in all of these but remove it anyway for safe
     * mesure
     */
    state->allocated = g_list_remove(state->allocated, queue);
    state->ordered_queues = g_list_remove(state->ordered_queues, queue);
    g_hash_table_remove(state->queues, queue);

    g_hash_table_destroy(queue->grouping_index);
    g_queue_free(queue->groupings);
    free((void *)queue->tags);
}

static struct gw_grouping_config *
grouping_get_config(struct gw_state *state, struct gw_grouping *grouping)
{
    struct gw_grouping_config *config;

    config = g_hash_table_lookup(state->data_streams_config,
                                 grouping->name);
    assert(config);

    return config;
}

static void gw_grouping_config_create(struct gw_state *state,
                                      struct gw_grouping *grouping)
{
    struct gw_grouping_config *config;

    config = g_hash_table_lookup(state->data_streams_config, grouping->name);
    if (config)
        return;

    config = xmalloc(sizeof(*config));
    // TODO load this from config if found
    config->max_streams = state->default_max_streams;
    config->current_streams = 0;
    config->grouping = xstrdup_safe(grouping->name);
    g_hash_table_insert(state->data_streams_config,
                        config->grouping,
                        config);
}

static void gw_grouping_config_remove(struct gw_state *state,
                                      struct gw_grouping *grouping,
                                      size_t n_streams)
{
    struct gw_grouping_config *config;

    config = grouping_get_config(state, grouping);

    assert(config->current_streams >= n_streams);
    config->current_streams -= n_streams;
    if (config->current_streams > 0)
        /* The grouping can still be in use in a different queue */
        return;

    g_hash_table_remove(state->data_streams_config,
                        grouping->name);
    free(config->grouping);
    free(config);
}

static struct gw_grouping *get_grouping(struct gw_state *state,
                                        struct gw_queue *queue,
                                        const char *name)
{
    struct gw_grouping *grouping;

    grouping = g_hash_table_lookup(queue->grouping_index, name);
    if (grouping)
        return grouping;

    grouping = gw_grouping_new(name);
    /* push at the end of the queue to preserve the order of arrival */
    queue_push(queue->groupings, grouping);
    g_hash_table_insert(queue->grouping_index, grouping->name, grouping);

    gw_grouping_config_create(state, grouping);

    return grouping;
}

static size_t __n_grouping_io(struct gw_state *state,
                              struct gw_grouping_config *config,
                              struct gw_grouping *grouping)
{
    return config->current_streams + n_writes_per_grouping(state->busy_devices,
                                                           grouping->name);
}

static size_t n_grouping_io(struct gw_state *state,
                            struct gw_grouping *grouping)
{
    struct gw_grouping_config *config;

    config = grouping_get_config(state, grouping);
    return __n_grouping_io(state, config, grouping);
}

static size_t grouping_io_maxed(struct gw_state *state,
                                struct gw_grouping *grouping)
{
    struct gw_grouping_config *config;

    config = grouping_get_config(state, grouping);
    return __n_grouping_io(state, config, grouping) >= config->max_streams;
}

static struct gw_request *gw_request_new(struct gw_queue *queue,
                                         struct req_container *reqc)
{
    struct gw_request *req = xmalloc(sizeof(*req));

    req->queue = queue;
    req->reqc = reqc;

    return req;
}

static void gw_request_free(struct gw_request *req)
{
    free(req);
}

static void gw_queue_push(struct gw_state *state,
                          struct gw_queue *queue,
                          struct req_container *reqc)
{
    struct gw_grouping *grouping;

    grouping = get_grouping(state, queue, walloc_grouping(reqc));
    assert(grouping);

    queue_push(grouping->requests, gw_request_new(queue, reqc));
}

static bool gw_queue_empty(struct gw_queue *queue)
{
    struct gw_grouping *first = g_queue_peek_head(queue->groupings);

    return !first || g_queue_is_empty(first->requests);
}

static void gw_grouping_finished(struct gw_state *state,
                                 struct gw_queue *queue,
                                 struct gw_grouping *grouping)
{
    size_t n_streams = 0;

    glist_foreach(iter, queue->streams) {
        struct gw_data_stream *stream = iter->data;

        if (stream->grouping == grouping) {
            n_streams++;
            /* stream is free'd here */
            gw_data_stream_finished(state, stream);
        }
    }

    gw_grouping_config_remove(state, grouping, n_streams);

    /* Since groupings can be run in parallel, this grouping might not be at the
     * head of the list.
     */
    g_queue_remove(queue->groupings, grouping);
    g_hash_table_remove(queue->grouping_index,
                        grouping->name);
    gw_grouping_free(grouping);

    if (gw_queue_empty(queue))
        gw_queue_destroy(state, queue);
}

static void handle_finished_io(struct gw_state *state)
{
    GPtrArray *free_devs = g_ptr_array_new();
    int i;

    /* Find devices that have finished their current I/O so that we can schedule
     * new requests to them. Devices are first added to a temporary list to avoid
     * removing while iterating over state->busy_devices.
     */
    for (i = 0; i < state->busy_devices->len; i++) {
        struct lrs_dev *dev = state->busy_devices->pdata[i];

        if (!dev_is_sched_ready(dev))
            continue;

        g_ptr_array_add(free_devs, dev);
    }

    /* XXX This could probably be only one loop if we start from the end of the
     * list so that we can remove devices as we iterate. This would also avoid
     * the linear scan of g_ptr_array_remove().
     */
    for (i = 0; i < free_devs->len; i++) {
        struct lrs_dev *dev = free_devs->pdata[i];

        g_ptr_array_remove(state->busy_devices, dev);
        g_ptr_array_add(state->free_devices, dev);
    }

    g_ptr_array_free(free_devs, TRUE);
}

static int gw_push_request(struct io_scheduler *io_sched,
                           struct req_container *reqc)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_queue *queue;

    handle_finished_io(state);

    assert(pho_request_is_write(reqc->req));

    queue = gw_find_queue(state, reqc);
    if (!queue)
        queue = gw_queue_create(state, reqc);

    assert(queue);
    gw_queue_push(state, queue, reqc);

    return 0;
}

/* Remove the first request of the stream's grouping.
 * This is called by .remove_request to remove the request that have just been
 * scheduled to devices.
 *
 * Even if a grouping is shared amongst several streams, each stream will be
 * managed sequentially by the scheduling API. This means that the first request
 * of the grouping has to be the same request returned by .peek_request and used
 * by .get_device_medium_pair to do the allocation. There is no situation where
 * the user of this API would call .remove_request by passing it a request that
 * was not just returned by .peek_request.
 */
static struct gw_request *gw_stream_pop(struct gw_state *state,
                                        struct gw_data_stream *stream)
{
    struct gw_request *req;

    req = queue_pop(stream->grouping->requests);
    if (g_queue_is_empty(stream->grouping->requests))
        gw_grouping_finished(state, stream->queue, stream->grouping);

    return req;
}

static struct gw_request *gw_stream_peek(struct gw_data_stream *stream)
{
    return queue_peek(stream->grouping->requests);
}

static int gw_remove_request(struct io_scheduler *io_sched,
                             struct req_container *reqc)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_data_stream *stream;
    struct gw_queue *queue;
    struct gw_request *req;

    /* .remove_request is only called when .peek_request returns a
     * req_container. If we are here, it means that it successfully
     * returned a request in which case both current_request and current_stream
     * must be set.
     */
    assert(state->current_request && state->current_stream);

    queue = state->current_request->queue;
    stream = state->current_stream;

    pho_debug("Request %p will be removed from grouped write scheduler", reqc);

    /* reset alloc index for the next requests in the queue */
    queue->alloc_index = queue->n_media;
    req = gw_stream_pop(state, stream);
    if (!req || (state->current_request && req != state->current_request))
        LOG_RETURN(-EINVAL,
                   "Expected request '%p' to be pop'ed from queue, found '%p'",
                   state->current_request, req);

    if (gw_queue_empty(req->queue))
        gw_queue_destroy(state, req->queue);

    gw_request_free(req);
    state->current_request = NULL;
    state->current_stream = NULL;

    return 0;
}

/* The request is still on the queue since we don't call .remove_request()
 * in this case. We just need to reset alloc_index here.
 */
static int gw_requeue(struct io_scheduler *io_sched,
                      struct req_container *reqc)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_queue *queue;

    queue = gw_find_queue(state, reqc);
    /* Since the request was not removed from the queue, the queue should
     * still exist.
     */
    assert(queue);

    queue->alloc_index = queue->n_media;
    return 0;
}

static size_t n_splits_for_this_queue(struct gw_queue *queue,
                                      const char *grouping)
{
    size_t n_splits = 0;

    glist_foreach(iter, queue->streams) {
        struct gw_data_stream *stream = iter->data;

        if (!strcmp(stream->grouping->name, grouping))
            n_splits++;
    }

    return n_splits;
}

static size_t gw_queue_n_requests(struct gw_queue *queue)
{
    size_t count = 0;

    queue_foreach(iter, queue->groupings) {
        struct gw_grouping *grouping = iter->data;

        count += g_queue_get_length(grouping->requests);
    }

    return count;
}

static bool can_split_queue(struct gw_state *state, struct gw_queue *queue)
{
    size_t min_reqs_per_stream;

    if (queue->n_media > state->free_devices->len)
        /* not enough devices to create a new stream right now */
        return false;

    /* We want at least min_reqs_before_split request per stream to manage
     * otherwise it is not worth spliting a queue.
     */
    min_reqs_per_stream =
        state->min_reqs_before_split * (g_list_length(queue->streams) + 1);
    if (gw_queue_n_requests(queue) < min_reqs_per_stream)
        return false;

    queue_foreach(iter, queue->groupings) {
        struct gw_grouping *grouping = iter->data;

        if (n_grouping_io(state, grouping) == 0 &&
            g_queue_get_length(grouping->requests) >=
                state->min_reqs_before_split)
            return true;
    }

    glist_foreach(iter, queue->streams) {
        struct gw_data_stream *stream = iter->data;
        struct gw_grouping *grouping = stream->grouping;
        size_t n_splits;

        if (grouping_io_maxed(state, grouping))
            continue;

        /* We must have at least min_reqs_before_split requests per stream */
        n_splits = n_splits_for_this_queue(stream->queue, grouping->name);
        if (n_splits * state->min_reqs_before_split <=
                g_queue_get_length(grouping->requests))
            return true;
    }

    return false;
}

static bool can_allocate_queue(struct gw_state *state,
                               struct gw_queue *queue)
{
    if (queue->n_media > state->free_devices->len)
        return false;

    queue_foreach(iter, queue->groupings) {
        struct gw_grouping *grouping = iter->data;

        /* We have at least one grouping not max'ed out. Since this queue is not
         * yet allocated, we can allocate it.
         */
        if (grouping_io_maxed(state, grouping))
            return true;
    }

    return false;
}

static struct gw_queue *gw_try_alloc_next_queue(struct gw_state *state)
{
    GList *first = g_list_first(state->ordered_queues);
    struct gw_queue *queue;

    if (!first)
        /* no more new requests not already allocated to a device */
        return NULL;

    queue = first->data;
    if (!can_allocate_queue(state, queue))
        /* Not enough devices, do not allocate. This scheduler relies on
         * smart scheduling policies prior to reaching the LRS. It will not
         * try to see if another queue in the list can fit right now.
         */
        return NULL;

    state->allocated = g_list_append(state->allocated, queue);
    state->ordered_queues = g_list_remove_link(state->ordered_queues, first);

    return queue;
}

static int gw_stream_next_grouping(struct gw_state *state,
                                   struct gw_queue *queue,
                                   struct gw_data_stream *stream)
{
    struct gw_grouping *not_started_grouping = NULL;
    struct gw_grouping *max_diff_grouping = NULL;
    struct gw_grouping_config *config;
    struct gw_grouping *best = NULL;
    ssize_t max_diff = -1;

    queue_foreach(iter, queue->groupings) {
        struct gw_grouping *grouping = iter->data;
        size_t n_splits;
        size_t n_ios;

        /* We prefer allocating a new grouping over splitting a grouping across
         * multiple media.
         */
        config = grouping_get_config(state, grouping);
        n_ios = __n_grouping_io(state, config, grouping);
        if (n_ios == 0) {
            best = grouping;
            goto alloc_grouping;
        }

        /* This grouping already has I/O being run but not on this queue. Take
         * the first non started grouping as it was the first received on the
         * queue.
         */
        n_splits = n_splits_for_this_queue(queue, grouping->name);
        if (n_splits == 0 && !not_started_grouping)
            not_started_grouping = grouping;

        if (not_started_grouping)
            /* once we have found a non started grouping, finding one that can
             * be split is irrelevent. But we can still look for a grouping that
             * hasn't been started at all.
             */
            continue;

        /* Try to find out whether this grouping can have another stream */
        if (n_ios >= config->max_streams ||
            g_queue_get_length(grouping->requests) <
                (n_splits + 1) * state->min_reqs_before_split)
            /* this grouping cannot be split */
            continue;

        /* We prefer allocating groupings that are the farthest away from their
         * max to prevent situations where a grouping takes all the devices
         * before others can be scheduled.
         */
        if (max_diff == -1 || max_diff < (config->max_streams - n_ios)) {
            max_diff_grouping = grouping;
            max_diff = (config->max_streams - n_ios);
        }
    }

    /* At this point best is NULL. */
    best = not_started_grouping ? : max_diff_grouping;
    if (!best)
        return -EBUSY;

alloc_grouping:
    stream->grouping = best;
    config->current_streams++;

    return 0;
}

static struct gw_data_stream *
gw_queue_split(struct gw_state *state, struct gw_queue *queue)
{
    struct gw_data_stream *new;

    new = gw_data_stream_new(queue);
    gw_stream_next_grouping(state, queue, new);
    queue->streams = g_list_append(queue->streams, new);

    return new;
}

static int gw_alloc_new_stream(struct gw_state *state)
{
    struct gw_data_stream *stream;
    struct gw_queue *queue = NULL;

    /* Find an allocated queue that can be split for the free devices */
    glist_foreach(iter, state->allocated) {
        struct gw_queue *q = iter->data;

        /* We may have a queue that could be split but we don't have enough
         * devices to do so right now. Since devices are associated to
         * streams until the current grouping is finished, we should try to
         * schedule a new queue right now if no queue can be split. Otherwise, a
         * device might be waiting a long time before a stream is finished.
         * Since we check for split before trying to allocate new queues, it is
         * possible that at some point in the futur we actually split this queue
         * before scheduling a new one (when another device becomes available
         * for instance).
         */
        if (can_split_queue(state, q)) {
            queue = q;
            break;
        }
    }

    if (!queue)
        /* If no queue can be split, find a new queue */
        queue = gw_try_alloc_next_queue(state);
    if (!queue)
        /* Not enough devices to allocate the next queue, try later */
        return 0;

    stream = gw_queue_split(state, queue);
    assert(stream->grouping);
    state->current_stream = stream;

    return 0;
}

static int gw_stream_next_request(struct gw_state *state,
                                  struct gw_data_stream *stream,
                                  struct req_container **reqc)
{
    struct gw_request *req = NULL;

    state->current_stream = stream;
    req = gw_stream_peek(state->current_stream);

    /* We should only be called here if the stream still has requests to manage
     */
    assert(req);

    *reqc = req->reqc;
    state->current_request = req;
    stream->queue->alloc_index = stream->queue->n_media;

    return 0;
}

static bool stream_busy(struct gw_queue *queue,
                        struct gw_data_stream *stream)
{
    size_t i;

    for (i = 0; i < queue->n_media; i++) {
        /* gw_streams_next_grouping and gw_alloc_new_stream should make sure
         * that all streams have a grouping at this point. If not, they are
         * removed from the queue.
         */
        assert(stream->grouping);

        if (!stream->devices[i])
            /* A stream that doesn't have all devices allocated cannot be busy */
            return false;

        /* At least one device is doing I/O, requests from this stream are
         * blocked until all devices are available.
         */
        if (!dev_is_sched_ready(stream->devices[i]))
            return true;
    }

    /* All devices are allocated and none are doing I/O write now, the stream
     * is not busy.
     */
    return false;
}

static struct gw_data_stream *find_idle_stream(struct gw_queue *queue)
{
    glist_foreach(iter, queue->streams) {
        if (stream_busy(queue, iter->data))
            continue;

        return iter->data;
    }

    return NULL;
}

/* Count the number of devices required to schedule I/O for all the streams.
 * Some streams may not yet have devices. The goal is to count how many devices
 * we need to compare it to the number of free devices.
 */
static size_t streams_device_count(struct gw_state *state)
{
    size_t count = 0;

    glist_foreach(iter, state->allocated) {
        struct gw_queue *queue = iter->data;

        count += g_list_length(queue->streams) * queue->n_media;
    }

    return count;
}

/**
 * 1. If we don't have allocated all the streams yet, try to find a new stream
 *    to allocate. It can be:
 *    - a grouping not already allocated in an already allocated queue
 *    - a new stream for a grouping already allocated
 *    - otherwise, the next queue if enough devices are available
 * 2. Find the first idle stream and schedule its first request
 */
static int gw_peek_request(struct io_scheduler *io_sched,
                           struct req_container **reqc)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_queue *queue = NULL;

    handle_finished_io(state);

    *reqc = NULL;

    if (streams_device_count(state) < io_sched->devices->len)
        gw_alloc_new_stream(state);

    /* Find first non busy queue to allocate a new request */
    glist_foreach(iter, state->allocated) {
        struct gw_data_stream *stream;

        queue = iter->data;
        stream = find_idle_stream(queue);
        if (!stream)
            continue;

        return gw_stream_next_request(state, stream, reqc);
    }

    return 0;
}

static struct string_array request_get_tags(struct gw_request *request)
{
    return (struct string_array) {
        .strings = request->reqc->req->walloc->media[0]->tags,
        .count = request->reqc->req->walloc->media[0]->n_tags,
    };
}

static bool medium_is_ready(struct gw_state *state,
                            struct gw_data_stream *stream,
                            size_t index)
{
    struct media_info *medium;
    struct string_array tags;
    struct gw_request *first;
    bool compatible;

    assert(stream->devices[index]);

    medium = atomic_dev_medium_get(stream->devices[index]);
    if (!medium)
        return false;

    /* All the requests here must have the same tags, we don't really care
     * which request we get. We just need the tags.
     */
    first = queue_peek(stream->grouping->requests);
    /* An empty stream should not be still there on peek request */
    assert(first);

    tags = request_get_tags(first);

    /* Ignore grouping here as this does not prevent the write */
    compatible = medium_is_write_compatible(medium, NULL, &tags, false);
    lrs_medium_release(medium);
    return compatible;
}

static int gw_get_device_medium_pair(struct io_scheduler *io_sched,
                                     struct req_container *reqc,
                                     struct lrs_dev **dev,
                                     size_t *index)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_data_stream *stream;
    struct gw_queue *queue;
    struct gw_request *req;
    int rc;

    assert(state->current_request && state->current_request->reqc == reqc);

    req = state->current_request;
    stream = state->current_stream;
    queue = req->queue;

    if ((queue->alloc_index == queue->n_media ||
        *index > queue->alloc_index) &&
        stream->devices[*index]) {
        /* Queue was allocated to a device for a previous request. Recheck
         * compatibility in case the state of the device or medium has changed.
         * No need to check the device status (e.g. failed), the upper layer will
         * do it for us.
         */
        if (medium_is_ready(state, stream, *index)) {
            /* XXX unit test where we lock a medium between 2 I/O */
            queue->alloc_index = *index;
            /* Reuse the same device. No need to update the stream as it was
             * already done on the first allocation.
             */
            *dev = stream->devices[*index];
            return 0;
        }
    }

    queue->alloc_index = *index;
    if (stream->devices[*index])
        /* The upper layer wants a different device, remove the previous one. */
        gw_data_stream_remove_device(state, stream->devices[*index], *index);

    // FIXME we need to look in the free_devices list otherwise we will
    // reallocated device already associated to a stream...
    rc = find_write_device(io_sched, req->reqc, dev, *index, false);
    if (rc)
        return rc;

    if (*dev)
        gw_data_stream_set_device(state, stream, *dev, *index);

    return 0;
}

/* The request might have been the last. If so the queue was removed
 * on .remove_request(). The other devices of the queue should still
 * be allocated. Simply reallocate the one that failed.
 */
static int gw_retry(struct io_scheduler *io_sched,
                    struct sub_request *sreq,
                    struct lrs_dev **dev)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_data_stream *stream;
    struct lrs_dev *old_device;
    int rc;

    /* retry is called after remove */
    assert(!state->current_request);

    old_device = sreq->reqc->params.rwalloc.respc->devices[sreq->medium_index];
    stream = gw_data_stream_remove_device(state, old_device,
                                          sreq->medium_index);

    // FIXME cf. other find_write_device
    rc = find_write_device(io_sched, sreq->reqc, dev,
                           sreq->medium_index, true);

    if (stream && *dev)
        gw_data_stream_set_device(state, stream, *dev, sreq->medium_index);

    return rc;
}

static void gw_add_device(struct io_scheduler *io_sched,
                          struct lrs_dev *new_device)
{
    struct gw_state *state = io_sched->private_data;
    size_t i;

    for (i = 0; i < io_sched->devices->len; i++) {
        struct lrs_dev *dev;

        dev = g_ptr_array_index(io_sched->devices, i);
        if (new_device == dev)
            return;
    }

    g_ptr_array_add(io_sched->devices, new_device);
    g_ptr_array_add(state->free_devices, new_device);
}

static struct lrs_dev **gw_get_device(struct io_scheduler *io_sched,
                                      size_t i)
{
    return (struct lrs_dev **)&io_sched->devices->pdata[i];
}

static int gw_remove_device(struct io_scheduler *io_sched,
                            struct lrs_dev *device)
{
    struct gw_state *state = io_sched->private_data;
    struct gw_data_stream *stream;

    /* remove the device from its queue if allocated */
    stream = g_hash_table_lookup(state->device_to_stream, device);
    if (stream) {
        size_t i;

        g_hash_table_remove(state->device_to_stream, device);
        for (i = 0; i < stream->queue->n_media; i++) {
          /* XXX Does this mean that a stream can be busy and have some devices
           * NULL???
           */
          if (stream->devices[i] == device)
            stream->devices[i] = NULL;
        }
    }

    g_ptr_array_remove(io_sched->devices, device);
    g_ptr_array_remove(state->free_devices, device);
    g_ptr_array_remove(state->busy_devices, device);

    return 0;
}

static int gw_exchange_device(struct io_scheduler *io_sched,
                              union io_sched_claim_device_args *args)
{
    struct gw_state *state = io_sched->private_data;
    struct lrs_dev *device_to_remove;
    struct lrs_dev *device_to_add;
    guint index;
    bool found;

    device_to_add = args->exchange.unused_device;
    found = g_ptr_array_find(state->free_devices,
                             args->exchange.desired_device,
                             &index);
    if (!dev_is_sched_ready(device_to_add) || !found)
        /* Device used, do not give it back. Note that this will block the device
         * until all the writes are done.
         *
         * XXX This is likely the read scheduler asking for this device.
         * We have two options here:
         * - block the read until the writes are finished which will incur less
         *   tape movement but slow down reads which may block applications;
         * - give priority to reads to increase object retrieval at the cost of
         *   write performance.
         *
         * The best option is probably to add a parameter to control this as this
         * is application dependant.
         */
        return 0;

    device_to_remove = g_ptr_array_index(state->free_devices, index);
    device_to_remove->ld_io_request_type &= ~io_sched->type;
    device_to_add->ld_io_request_type = io_sched->type;
    gw_add_device(io_sched, device_to_add);
    g_ptr_array_remove(io_sched->devices, device_to_remove);

    return 0;
}

static struct lrs_dev *gw_find_device_to_remove(struct io_scheduler *io_sched,
                                                const char *techno)
{
    struct gw_state *state = io_sched->private_data;
    size_t i;

    for (i = 0; i < state->free_devices->len; i++) {
        struct lrs_dev *device = g_ptr_array_index(state->free_devices, i);

        if (strcmp(techno, device->ld_technology))
            continue;

        return device;
    }

    return NULL;
}

static int gw_claim_device(struct io_scheduler *io_sched,
                           enum io_sched_claim_device_type type,
                           union io_sched_claim_device_args *args)
{
    switch (type) {
    case IO_SCHED_BORROW:
        return -ENOTSUP;
    case IO_SCHED_EXCHANGE:
        return gw_exchange_device(io_sched, args);
    case IO_SCHED_TAKE:
        /* give back device to dispatch algorithm (e.g. faire_share) even if in
         * the middle of an I/O. If this goes to the read scheduler we will
         * likely have to reload another tape later for the current I/O.
         */
        args->take.device =
            gw_find_device_to_remove(io_sched, args->take.technology);

        if (!args->take.device)
            return -ENODEV;

        gw_remove_device(io_sched, args->take.device);

        return 0;
    default:
        return -EINVAL;
    }

    return 0;
}

struct io_scheduler_ops IO_SCHED_GROUPED_WRITE_OPS = {
    .init                   = gw_init,
    .fini                   = gw_fini,
    .push_request           = gw_push_request,
    .remove_request         = gw_remove_request,
    .requeue                = gw_requeue,
    .peek_request           = gw_peek_request,
    .get_device_medium_pair = gw_get_device_medium_pair,
    .retry                  = gw_retry,
    .add_device             = gw_add_device,
    .get_device             = gw_get_device,
    .remove_device          = gw_remove_device,
    .claim_device           = gw_claim_device,
};
