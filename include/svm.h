/*
 * svm.h
 *
 *  Created on: Apr 19, 2022
 *      Author: yuanhao
 */

#ifndef INC_SVM_H_
#define INC_SVM_H_

#ifdef __cplusplus
extern "C"{
#endif
#include "stdbool.h"
#include "stdint.h"
#define DATA_FEATURES_DIMENSIONS 6          //输入数据的特征

struct PREDICET_PARAM{
	const uint8_t svm;
	const uint8_t ann;
	uint8_t pred_model;      //预测模型选择 0：SVM  1：ANN
};

extern struct PREDICET_PARAM predict_parm;

struct SVM_MODEL
{
	/* data */
	const double coefs[DATA_FEATURES_DIMENSIONS];  //系数
	const double b;                                //截距
};

int8_t svm_prediction(uint16_t *arr_in, uint8_t hair_status);

#ifdef __cplusplus
}
#endif


#endif /* INC_SVM_H_ */
