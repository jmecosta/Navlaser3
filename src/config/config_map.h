/*
 * config_map.h
 *
 *  Created on: 27 Mar 2011
 *      Author: jmecosta
 */

#ifndef CONFIG_MAP_H_
#define CONFIG_MAP_H_

class config_map
{
protected:
	double LINE_VALIDATION_GATE;
	double G_NO_MATCH;
public:
	config_map();
	virtual ~config_map();
	double getLineValGate () { return LINE_VALIDATION_GATE; };
};

#endif /* CONFIG_MAP_H_ */
