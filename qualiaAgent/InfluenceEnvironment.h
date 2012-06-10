/*
 * InfluenceEnvironment.h
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

#ifndef INFLUENCEENVIRONMENT_H_
#define INFLUENCEENVIRONMENT_H_

#include "core/Environment.h"
#include "rl/RLObservation.h"
#include "util/Random.h"

#include <mapper/mapper.h>

class InfluenceEnvironment : public Environment {
public:
//  mapper_device dev;
//  const char* devNamePrefix;
//  bool autoConnect;
//  int devInitialPort;
//  mapper_signal outsig;
  //mapper_signal outsigX, outsigY;
  RLObservation currentObservation;
  int observationDim, actionDim;

  float pos[2];
  float vel[2];

  InfluenceEnvironment(int observationDim, int actionDim);
  virtual ~InfluenceEnvironment();

  virtual void init();
  virtual Observation* start();
  virtual Observation* step(const Action* action);

//  static void updateInput(mapper_signal sig, mapper_db_signal props,
//                          mapper_timetag_t *timetag, float *value);


};

#endif /* INFLUENCEENVIRONMENT_H_ */
