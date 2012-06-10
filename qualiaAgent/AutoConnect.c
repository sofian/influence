#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <mapper/mapper.h>
#include "AutoConnect.h"

struct _autoConnectState
{
    int seen_srcdest_link;
    int seen_destsrc_link;
    char *vector_device_name;
    char *xagora_device_name;
    int connected;

    mapper_device dev;
    mapper_monitor mon;
} autoConnectState;

mapper_signal sig_x = 0, sig_y = 0;

float obs[5] = {0,0,0,0,0};

#define WIDTH  640
#define HEIGHT 480

int id = 0;

void make_connections()
{
    char signame1[1024], signame2[1024];
    struct _autoConnectState *acs = &autoConnectState;

    sprintf(signame1, "%s/node/%d/observation",
            acs->vector_device_name, mdev_ordinal(acs->dev));

    sprintf(signame2, "%s/observation", mdev_name(acs->dev));

    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);

    sprintf(signame1, "%s/position", mdev_name(acs->dev));

    sprintf(signame2, "%s/node/%d/position",
            acs->vector_device_name, mdev_ordinal(acs->dev));

    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);
//    sprintf(signame1, "%s/position/x", mdev_name(acs->dev));
//
//    sprintf(signame2, "%s/node/%d/position/x",
//            acs->vector_device_name, mdev_ordinal(acs->dev));
//
//    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);
//
//    sprintf(signame2, "%s/X_Butterfly%d",
//            acs->xagora_device_name, mdev_ordinal(acs->dev));
//
//    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);
//
//    sprintf(signame1, "%s/position/y", mdev_name(acs->dev));
//
//    sprintf(signame2, "%s/node/%d/position/y",
//            acs->vector_device_name, mdev_ordinal(acs->dev));
//
//    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);
//
//    sprintf(signame2, "%s/Z_Butterfly%d",
//            acs->xagora_device_name, mdev_ordinal(acs->dev));
//
//    mapper_monitor_connect(acs->mon, signame1, signame2, 0, 0);
}

void signal_handler(mapper_signal msig,
                    mapper_db_signal props,
                    mapper_timetag_t *timetag,
                    void *value)
{
    memcpy(obs, value, sizeof(float)*4);
    printf("observation: %f, %f, %f, %f\n",
           obs[0], obs[1], obs[2], obs[3]);
}

void link_db_callback(mapper_db_link record,
                      mapper_db_action_t action,
                      void *user)
{
    struct _autoConnectState *acs = (struct _autoConnectState*)user;

    if (!acs->vector_device_name || !mdev_name(acs->dev))
        return;

    if (action == MDB_NEW || action == MDB_MODIFY) {
        if (strcmp(record->src_name, acs->vector_device_name)==0
            &&
            strcmp(record->dest_name, mdev_name(acs->dev))==0)
        {
            acs->seen_srcdest_link = 1;
        }

        if (strcmp(record->src_name, mdev_name(acs->dev))==0
            &&
            strcmp(record->dest_name, acs->vector_device_name)==0)
        {
            acs->seen_destsrc_link = 1;
        }
    }

    if (acs->seen_srcdest_link && acs->seen_destsrc_link)
    {
//        float mn=0, mx=1;
//        mdev_add_input(acs->dev, "observation", 4, 'f', "norm", &mn, &mx,
//                       signal_handler, 0);
//        int imn=0, imx=640;
//        sig_x = mdev_add_output(acs->dev, "position/x", 1, 'i', 0, &imn, &imx);
//        imn=0; imx=480;
//        sig_y = mdev_add_output(acs->dev, "position/y", 1, 'i', 0, &imn, &imx);

        acs->connected = 1;
    }
}

mapper_device autoConnectDevice(mapper_device dev)
{
    struct _autoConnectState *acs = &autoConnectState;
    memset(acs, 0, sizeof(struct _autoConnectState));

    acs->dev = dev;

    while (!mdev_ready(acs->dev)) {
         mdev_poll(acs->dev, 100);
     }

     printf("ordinal: %d\n", mdev_ordinal(acs->dev));
     fflush(stdout);

     acs->mon = mapper_monitor_new(0, 0);
     mapper_db db = mapper_monitor_get_db(acs->mon);

     mapper_db_add_link_callback(db, link_db_callback, acs);
     int i=0;
     while (i++ < 10) {
         mdev_poll(acs->dev, 100);
         mapper_monitor_poll(acs->mon, 0);
     }

     mapper_db_device *dbdev = mapper_db_match_devices_by_name(db, "vector");
     if (dbdev) {
         acs->vector_device_name = strdup((*dbdev)->name);
         mapper_monitor_link(acs->mon, (*dbdev)->name, mdev_name(acs->dev));
         mapper_monitor_link(acs->mon, mdev_name(acs->dev), (*dbdev)->name);

         mapper_monitor_request_links_by_name(acs->mon, (*dbdev)->name);
     }
     mapper_db_device_done(dbdev);

//     dbdev = mapper_db_match_devices_by_name(db, "XAgora_receiver");
//     if (dbdev) {
//         acs->xagora_device_name = strdup((*dbdev)->name);
//         mapper_monitor_link(acs->mon, mdev_name(acs->dev), (*dbdev)->name);
//
//         mapper_monitor_request_links_by_name(acs->mon, (*dbdev)->name);
//     }
//     mapper_db_device_done(dbdev);

     i=0;
     while (i++ < 1000 && !acs->connected) {
         mdev_poll(acs->dev, 100);
         mapper_monitor_poll(acs->mon, 0);
     }

     if (!acs->connected) {
         mdev_free(acs->dev);
         mapper_monitor_free(acs->mon);
         return 0;
     }

     make_connections();

     return acs->dev;
}

void autoDisconnectDevice()
{
  struct _autoConnectState *acs = &autoConnectState;
  if (acs->vector_device_name) {
      mapper_monitor_unlink(acs->mon,
                            acs->vector_device_name,
                            mdev_name(acs->dev));
      free(acs->vector_device_name);
  }
  if (acs->xagora_device_name) {
      mapper_monitor_unlink(acs->mon,
                            mdev_name(acs->dev),
                            acs->xagora_device_name);
      free(acs->xagora_device_name);
  }
  if (acs->dev)
      mdev_free(acs->dev);
  memset(acs, 0, sizeof(struct _autoConnectState));
}

//int main()
//{
//    mapper_device dev = autoConnect();
//    if (!dev)
//        return 1;
//
//    mdev_poll(dev, 1000);
//    autoDisconnect();
//    return 0;
//}
