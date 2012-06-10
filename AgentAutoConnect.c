#include "AgentAutoConnect.h"

void make_influence_connections()
{
    char signame1[1024], signame2[1024];
    struct _agentInfo *info = &agentInfo;

    sprintf(signame1, "%s/node/%d/observation",
            info->influence_device_name, mdev_ordinal(info->dev));

    sprintf(signame2, "%s/observation", mdev_name(info->dev));

    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);

    sprintf(signame1, "%s/position", mdev_name(info->dev));

    sprintf(signame2, "%s/node/%d/position",
            info->influence_device_name, mdev_ordinal(info->dev));

    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);
}

void make_xagora_connections()
{
    char signame1[1024], signame2[1024];
    struct _agentInfo *info = &agentInfo;

    sprintf(signame2, "%s/Butterfly%d",
            info->xagora_device_name, mdev_ordinal(info->dev));

    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);
}

void signal_handler(mapper_signal msig,
                    mapper_db_signal props,
                    mapper_timetag_t *timetag,
                    void *value)
{
    memcpy(obs, value, sizeof(float)*2);
    printf("observation: %f, %f\n", obs[0], obs[1]);
}

void dev_db_callback(mapper_db_device record,
                     mapper_db_action_t action,
                     void *user)
{
    // if we see /influence.1 or /XAgora.1, send /link message
    struct _agentInfo *info = (struct _agentInfo*)user;

    if (action == MDB_NEW) {
        if (strcmp(record->name, info->influence_device_name)==0) {
            mapper_monitor_link(info->mon, mdev_name(info->dev),
                                record->name);
            mapper_monitor_link(info->mon, record->name,
                                mdev_name(info->dev));
        }
        else if (strcmp(record->name, info->xagora_device_name)==0)
            mapper_monitor_link(info->mon, mdev_name(info->dev),
                                record->name);
    }
    else if (action == MDB_REMOVE) {
        if (strcmp(record->name, info->influence_device_name)==0) {
            mapper_monitor_unlink(info->mon, mdev_name(info->dev),
                                  record->name);
            info->linked_influence = 0;
        }
    }
}

void link_db_callback(mapper_db_link record,
                      mapper_db_action_t action,
                      void *user)
{
    // if we see our links, send /connect messages
    struct _agentInfo *info = (struct _agentInfo*)user;

    if (action == MDB_NEW) {
        if (((strcmp(record->src_name, info->influence_device_name)==0) &&
             (strcmp(record->dest_name, mdev_name(info->dev))==0)) ||
            ((strcmp(record->dest_name, info->influence_device_name)==0) &&
             (strcmp(record->src_name, mdev_name(info->dev))==0))) {
            info->linked_influence++;
            if (info->linked_influence >= 2)
                make_influence_connections();
        }
        else if ((strcmp(record->src_name, mdev_name(info->dev))==0) &&
              (strcmp(record->dest_name, info->xagora_device_name)==0)) {
            make_xagora_connections();
        }
    }
    else if (action == MDB_REMOVE) {
        if ((strcmp(record->src_name, info->influence_device_name)==0) ||
            (strcmp(record->dest_name, info->influence_device_name)==0))
            info->linked_influence--;
    }
}

struct _agentInfo *agentInit()
{
    obs[0] = obs[1] = 0;
    id = 0;

    struct _agentInfo *info = &agentInfo;
    memset(info, 0, sizeof(struct _agentInfo));

    info->influence_device_name = strdup("/influence.1");
    info->xagora_device_name = strdup("/XAgora_receiver.1");

    info->admin = mapper_admin_new(0, 0, 0);

    // add device
    info->dev = mdev_new("agent", 9000 + id, info->admin);
    while (!mdev_ready(info->dev)) {
        mdev_poll(info->dev, 100);
    }
    printf("ordinal: %d\n", mdev_ordinal(info->dev));
    fflush(stdout);

    // add monitor and monitor callbacks
    info->mon = mapper_monitor_new(info->admin, 0);
    info->db  = mapper_monitor_get_db(info->mon);
    mapper_db_add_device_callback(info->db, dev_db_callback, info);
    mapper_db_add_link_callback(info->db, link_db_callback, info);

    // add signals
    float mn=-1, mx=1;
    mdev_add_input(info->dev, "observation", 2, 'f', "norm", &mn, &mx,
                   signal_handler, 0);
    int imn=0, imx=WIDTH;
    info->sig_pos = mdev_add_output(info->dev, "position", 2, 'i', 0, &imn, &imx);
    info->sig_gain = mdev_add_output(info->dev, "gain", 1, 'f',
                               "normalized", &mn, &mx);
    mx = 0.9;
    info->sig_fade = mdev_add_output(info->dev, "fade", 1, 'f', "normalized", &mn, &mx);
    mn = -1.5;
    mx = 1.5;
    info->sig_spin = mdev_add_output(info->dev, "spin", 1, 'f', "radians", &mn, &mx);
    mn = -3.1415926;
    mx = 3.1315926;
    info->sig_dir = mdev_add_output(info->dev, "direction", 1, 'f', "radians", &mn, &mx);
    mn = -1;
    info->sig_flow = mdev_add_output(info->dev, "flow", 1, 'f', "noramlized", &mn, &mx);

    return info;
}

void agentLogout()
{
    struct _agentInfo *info = &agentInfo;

    if (info->influence_device_name) {
        mapper_monitor_unlink(info->mon,
                              info->influence_device_name,
                              mdev_name(info->dev));
        free(info->influence_device_name);
    }
    if (info->xagora_device_name) {
        mapper_monitor_unlink(info->mon,
                              mdev_name(info->dev),
                              info->xagora_device_name);
        free(info->xagora_device_name);
    }
    if (info->dev)
        mdev_free(info->dev);
    if (info->db) {
        mapper_db_remove_device_callback(info->db, dev_db_callback, info);
        mapper_db_remove_link_callback(info->db, link_db_callback, info);
    }
    if (info->mon)
        mapper_monitor_free(info->mon);
    if (info->admin) {
        mapper_admin_free(info->admin);
    }
    memset(info, 0, sizeof(struct _agentInfo));
}
