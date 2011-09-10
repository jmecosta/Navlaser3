/*
 * config_movement.h
 *
 *  Created on: 27 Mar 2011
 *      Author: jmecosta
 */

#ifndef CONFIG_MOVEMENT_H_
#define CONFIG_MOVEMENT_H_

class config_movement
{
protected:
	// variable to define the model type Level 1 propagator
	int MODEL_TYPE;
public:
	config_movement();
	virtual ~config_movement();
};

#endif /* CONFIG_MOVEMENT_H_ */
