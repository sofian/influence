/*
 * InfluenceEnvironment.cpp
 *
 * (c) 2012 Sofian Audry -- info(@)sofianaudry(.)com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "InfluenceEnvironment.h"

#include "../AgentAutoConnect.h"

#include <unistd.h>
#include <stdio.h>

struct _agentInfo *info;


#include <assert.h>

InfluenceEnvironment::InfluenceEnvironment(int observationDim_, int actionDim_)
  : currentObservation(observationDim_), observationDim(observationDim_), actionDim(actionDim_) {
}

InfluenceEnvironment::~InfluenceEnvironment() {
  agentLogout();
//  autoDisconnectDevice();
//  if (dev)
//    mdev_free(dev);
}

void InfluenceEnvironment::init() {
  printf("Init env\n");
  info = agentInit();
  printf("Assert\n");
  assert (info->dev);
  printf("Done\n");
}

Observation* InfluenceEnvironment::start() {
  printf("Starting env\n");
  pos[0] = rand()%WIDTH/2+WIDTH/4;
  pos[1] = rand()%WIDTH/2+WIDTH/4;
  vel[0] = vel[1] = 0;

  printf("COCO: %p\n", info);
  mapper_monitor_poll(info->mon, 0);
  //mdev_poll(info->dev, 10);

  // Send position.
  printf("Update\n");
  int p[2];
  p[0] = (int)pos[0];
  p[1] = (int)pos[1];
  msig_update(info->sig_pos, p);

  // Wait for response.
  //mapper_monitor_poll(info->mon, 0);
  printf("Polling\n");
  mdev_poll(info->dev, 1000);

  printf("Starting env\n");
  return &currentObservation;
}

Observation* InfluenceEnvironment::step(const Action* action) {
  printf("Stepping env\n");
  // Update velocity depending on chosen action.
  float gain = 2;
  float limit = 1;

  float magnet = (action->actions[0] == 0 ? -1 : +1);
  printf("Action chosen: %d\n", action->actions[0]);
  //vel[0] = vel[1] = gain;
  //magnet = 1;
  vel[0] += magnet * gain * (currentObservation[0] - currentObservation[2]);
  vel[1] += magnet * gain * (currentObservation[1] - currentObservation[3]);

  pos[0] += vel[0];
  pos[1] += vel[1];

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

  // Send position.
  //int x = (int)pos[0];
  //int y = (int)pos[1];
  //msig_update(outsigX, &x);
  //msig_update(outsigY, &y);
  int p[2];
  p[0] = (int)pos[0];
  p[1] = (int)pos[1];
  msig_update(info->sig_pos, p);
  
  // Wait for retroaction.
  while (! mdev_poll(info->dev, 10) );

  // Compute reward.

  memcpy(currentObservation.observations, obs, sizeof(float)*2);
  currentObservation.reward = sqrt( vel[0]*vel[0] + vel[1]*vel[1] ); // go fast

  printf("--> receiving reward = %f, data = %f ...\n", currentObservation.reward, currentObservation[0]);

  return &currentObservation;
}

//void InfluenceEnvironment::updateInput(mapper_signal sig, mapper_db_signal props,
//                                        mapper_timetag_t *timetag, float *value) {
//  printf("update input called %f\n", *value);
//  RLObservation& obs = ((InfluenceEnvironment*)props->user_data)->currentObservation;
//  for (unsigned int i=0; i<obs.dim; i++)
//    obs[i] = value[i];
//}
