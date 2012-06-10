#ifndef AGENTAUTOCONNECT
#define AGENTAUTOCONNECT

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <mapper/mapper.h>

struct _agentInfo
{
    char *influence_device_name;
    char *xagora_device_name;

    int linked_influence;
    int connected;

    mapper_admin admin;
    mapper_device dev;
    mapper_monitor mon;
    mapper_db db;

    mapper_signal sig_pos,
                  sig_gain,
                  sig_spin,
                  sig_fade,
                  sig_dir,
                  sig_flow;
} agentInfo;


float obs[2];

#define WIDTH  500
#define HEIGHT 500

int id;

void make_influence_connections();

void make_xagora_connections();

void signal_handler(mapper_signal msig,
                    mapper_db_signal props,
                    mapper_timetag_t *timetag,
                    void *value);

void dev_db_callback(mapper_db_device record,
                     mapper_db_action_t action,
                     void *user);

void link_db_callback(mapper_db_link record,
                      mapper_db_action_t action,
                      void *user);

struct _agentInfo *agentInit();

void agentLogout();

#endif AGENTAUTOCONNECT

