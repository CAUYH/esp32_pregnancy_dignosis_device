#ifdef __cplusplus
extern "C"
{
#endif
#include "svm.h"

	struct PREDICET_PARAM predict_parm =
		{
			0,
			1,
			0

	};                                          
	//剃毛SVM模型
	struct SVM_MODEL svm_cut_hair = {
		{0.03894951, 0.01457945, -0.0215089,  -0.02311201 ,-0.02710561 , 0.03023509},
		20.03718095413438,
	};
	//未剃毛SVM模型
	struct SVM_MODEL svm_no_cut_hair = {
		{-0.00520748,0.00459994,-0.01411506,0.01286198,-0.01181688,0.00171487},
		 12.442865480579417,
	};
	/*
	 * svm模型预测函数
	 * 计算公式: y=w*xi+b
	 *           y>0 标签为1
	 *           y<0 标签为-1
	 * :param *arr_in   输入数据指针
	 * :param hair_status   剃毛状态
	 * :return  lable   预测标签
	 */
	int8_t svm_prediction(uint16_t *arr_in, uint8_t hair_status)
	{
		float realResult = 0.0;
		float y = 0.0;
		for (uint8_t i = 0; i < DATA_FEATURES_DIMENSIONS; i++)
		{ //计算内积
			if (hair_status)
			{ //计算剃毛模型
				realResult += arr_in[i] * svm_cut_hair.coefs[i];
			}
			else
			{ //计算未剃毛模型
				realResult += arr_in[i] * svm_no_cut_hair.coefs[i];
			}
		}
		if (hair_status) //加剃毛模型截距
		{
			y = realResult + svm_cut_hair.b;
		}
		else //加未剃毛模型截距
		{
			y = realResult + svm_no_cut_hair.b;
		}
		if (y > 0)
		{

			return 1;
		}
		else if (y < 0)
		{

			return -1;
		}
		else
		{
			return 0;
		}
	}

#ifdef __cplusplus
}
#endif
