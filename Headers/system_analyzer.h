/*
 * system_analyzer.h
 *
 *  Created on: Nov 19, 2014
 *      Author: abamberger
 */

#ifndef SYSTEM_ANALYZER_H_
#define SYSTEM_ANALYZER_H_

#include "spi.h"

void InitSystemAnalyzer();
int16 SystemAnalyzerSendReceive(int16 data);


#endif /* SYSTEM_ANALYZER_H_ */
