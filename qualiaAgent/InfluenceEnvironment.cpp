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
//  float gain = 2;
  float limit = 1;
//  float gain = 1;

  printf("actions: %d %d (%f, %f)\n", (action->actions[0]), (action->actions[1]), vel[0], vel[1]);
  for (int i=0; i<2; i++)
    vel[i] += ((float)action->actions[i] - 1.0f) * 0.1;

//  float magnet = (action->actions[0] == 0 ? -1 : +1);
//  printf("Action chosen: %d\n", action->actions[0]);
//  float rotationGain = 0.1;
//  float rot = (action->actions[0] - 1) * rotationGain;
//
//  rot = 1;
//  //  if (action->actions[0] == 0) // left
//  //    theta -= 0.1;
//  //  else if (action->actions[0] == 2) // left
//  //    theta += 0.1;
//
//  vel[0] += cos(rot);
//  vel[1] += sin(rot);

  // Compute velocity in polar.

  /*
  if (vel[0] == 0) vel[0] = 0.000000000001;
  float theta = atan(vel[1]/vel[0]);
  float r     = sqrt(vel[0]*vel[0] + vel[1]*vel[1]);

  theta += 2*M_PI;

  printf("%f %f\n", theta, r);
  theta += 0.01;
//  if (action->actions[0] == 0) // left
//    theta -= 0.1;
//  else if (action->actions[0] == 2) // left
//    theta += 0.1;

  vel[0] = r * cos(theta);
  vel[1] = r * sin(theta);*/
//  vel[0] += magnet * gain * (currentObservation[0] - currentObservation[2]);
//  vel[1] += magnet * gain * (currentObservation[1] - currentObservation[3]);

  pos[0] += vel[0];
  pos[1] += vel[1];

  if (vel[0] >  limit) vel[0] =  limit;
  if (vel[0] < -limit) vel[0] = -limit;
  if (vel[1] >  limit) vel[1] =  limit;
  if (vel[1] < -limit) vel[1] = -limit;

//  pos[0] = (int) (pos[0]) % (WIDTH-1);
//  pos[1] = (int) (pos[1]) % (HEIGHT-1);
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

  memcpy(currentObservation.observations, obs, sizeof(float)*4);
  //currentObservation.reward = sqrt( vel[0]*vel[0] + vel[1]*vel[1] ); // go fast
  float rew1 = 0;
  for (int i=0; i<(int)currentObservation.dim; i++)
    rew1 += currentObservation[i];
  rew1 /= currentObservation.dim;

  float diffX = pos[0] - WIDTH/2;
  float diffY = pos[1] - HEIGHT/2;
  float dist = sqrt( diffX*diffX + diffY*diffY ) / (WIDTH / 2);
  float rew2 = (dist > 0.5 ? 10 : 1) * (-dist);

  // stay away from the center
  currentObservation.reward = rew1 + rew2;

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
