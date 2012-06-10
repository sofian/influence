#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <mapper/mapper.h>

#include "AgentAutoConnect.h"

int done = 0;

void ctrlc(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    if (argc > 1)
        id = atoi(argv[1]);

    signal(SIGINT, ctrlc);

    struct _agentInfo *info = agentInit();
    if (!info->dev)
        goto done;

    float pos[2];
    pos[0] = rand()%WIDTH/2+WIDTH/4;
    pos[1] = rand()%HEIGHT/2+HEIGHT/4;
    float gain = 0.2;
    float damping = 0.9;
    float limit = 1;
    float vel[2] = {0,0};
    int counter = 0;

    while (!done) {
        mapper_monitor_poll(info->mon, 0);
        if (mdev_poll(info->dev, 10)) {
            vel[0] += obs[0] * gain;
            vel[1] += obs[1] * gain;
            pos[0] += vel[0];
            pos[1] += vel[1];
            vel[0] *= damping;
            vel[1] *= damping;

            if (vel[0] >  limit) vel[0] =  limit;
            if (vel[0] < -limit) vel[0] = -limit;
            if (vel[1] >  limit) vel[1] =  limit;
            if (vel[1] < -limit) vel[1] = -limit;

            if (pos[0] < 0) {
                pos[0] = 0;
                vel[0] *= -0.95;
            }

            if (pos[0] >= WIDTH) {
                pos[0] = WIDTH-1;
                vel[0] *= -0.95;
            }

            if (pos[1] < 0) {
                pos[1] = 0;
                vel[1] *= -0.95;
            }

            if (pos[1] >= HEIGHT) {
                pos[1] = HEIGHT-1;
                vel[1] *= -0.95;
            }

            int p[2];
            p[0] = (int)pos[0];
            p[1] = (int)pos[1];
            msig_update(info->sig_pos, p);

            counter = 0;
        }
        else {
            if (counter++ > 100) {
                int p[2];
                p[0] = (int)pos[0];
                p[1] = (int)pos[1];
                msig_update(info->sig_pos, p);
                counter = 0;
            }
        }
    }

done:
    agentLogout();
    return 0;
}
