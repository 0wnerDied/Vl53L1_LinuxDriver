
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file ipp_linux.c kernel side implementation of vl53l1 protected processing
 *
 *  @date  Sep 1, 2016
 *  @author : imaging
 *
 *  @ingroup ipp_dev
 */

#include "stmvl53l1.h"

#define IPP_ERR_CODE (VL53L1_ERROR_PLATFORM_SPECIFIC_START-1)

static int stmvl53l1_ipp_do_wrapper(struct stmvl53l1_data *data,
	struct ipp_work_t *pin, struct ipp_work_t *pout, int payload_out)
{
	int rc;

	if (data->ipp.buzy) {
		vl53l1_errmsg("try exec new ipp but still buzy on previous");
		/*  TODO shall we discard it and push new ? */
		rc = IPP_ERR_CODE;
		goto done;
	}
	WARN_ON(pin->payload > IPP_WORK_MAX_PAYLOAD);
	stmvl531_ipp_tim_start(data);
	rc = stmvl53l1_ipp_do(data, pin, pout);
	if (rc != 0) {
		vl53l1_errmsg("stmvl53l1_ipp_do err %d\n", rc);
		rc = IPP_ERR_CODE;
		goto done;
	}
	vl53l1_dbgmsg("ipp ok \n");
	/* check what we got back if valid answer error etc */
	if (pout->status) {
		vl53l1_errmsg("ipp error status %d from user", pout->status);
		if (pout->status >= stmvl53l1_ipp_status_proc_code)
			rc = pout->status & (stmvl53l1_ipp_status_proc_code-1);
		else
			rc = IPP_ERR_CODE;
		goto done;
	}
	WARN_ON(pout->payload > IPP_WORK_MAX_PAYLOAD);
	if (pout->payload != payload_out) {
		/* bad formated answer */
		vl53l1_errmsg("bad payload %d != %d in ipp work back",
				pout->payload, payload_out);
		rc = IPP_ERR_CODE;
		goto done;
	}
	stmvl531_ipp_tim_stop(data);
	stmvl531_ipp_stat(data, "ipp #%5x to=%3d fm=%3d in %5ld us",
			pin->xfer_id, pin->payload,
			pout->payload,
			stmvl531_ipp_time(data));

	rc = 0;
done:

	return rc;
}

/**
 * @file   vl53l1_platform_ipp.h
 *
 * @brief  EwokPlus25 IPP Wrapper Functions
 */

/**
 * @brief  IPP Wrapper call for histogram post processing
 *
 *
 * @param[in]    Dev                       : Device handle
 * @param[in]    pdmax_cal                 : DMAX calibration data
 * @param[in]    pdmax_cfg                 : DMAX configuration data
 * @param[in]    ppost_cfg                 : VL53L1_hist_post_process_config_t
 * @param[in]    pbins                     : Input histogram raw bin data
 * @param[in]    pxtalk                    : Cross talk histogram data
 * @param[in]    pArea1                    : Extern. allocated to save stack
 * @param[in]    pArea1                    : Extern. allocated to save stack
 * @param[out]   presults                  : Output VL53L1_range_results_t
 *                                           structure
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_ipp_hist_process_data(
	VL53L1_DEV dev,
	VL53L1_dmax_calibration_data_t    *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
	VL53L1_hist_post_process_config_t *ppost_cfg,
	VL53L1_histogram_bin_data_t       *pbins,
	VL53L1_xtalk_histogram_data_t     *pxtalk,
	uint8_t                           *pArea1,
	uint8_t                           *pArea2,
	uint8_t                           *phisto_merge_nb,
	VL53L1_range_results_t            *presults)
{
	struct stmvl53l1_data *data = (struct stmvl53l1_data *)
		container_of(dev, struct stmvl53l1_data, stdev);
	struct ipp_work_t *pout = &data->ipp.work_out;
	struct ipp_work_t *pin = &data->ipp.work;
	int rc;
	VL53L1_range_results_t *presults_ipp;

	IPP_SERIALIZE_VAR;

	/* These external workspace input params are useless with IPP managed in
	 * the daemon because the histogram processing function is actually
	 * allocating its data in USER stack within the daemon not the kernel one
	 */
	(void)pArea1;
	(void)pArea2;

	/* setup pin */
	IPP_SERIALIZE_START(pin->data, 6);
	IPP_SET_ARG_PTR(pin->data, 0, pdmax_cal);
	IPP_SET_ARG_PTR(pin->data, 1, pdmax_cfg);
	IPP_SET_ARG_PTR(pin->data, 2, ppost_cfg);
	IPP_SET_ARG_PTR(pin->data, 3, pbins);
	IPP_SET_ARG_PTR(pin->data, 4, pxtalk);
	IPP_SET_ARG_PTR(pin->data, 5, phisto_merge_nb);
	pin->payload = IPP_SERIALIZE_PAYLAOD();
	pin->process_no = stmvl53l1_ipp_cal_hist;

	/* setup pout */
	IPP_SERIALIZE_START(pout->data, 1);
	IPP_OUT_ARG_PTR(pout->data, 0, presults_ipp);

	/* do ipp */
	rc = stmvl53l1_ipp_do_wrapper(data, pin, pout, IPP_SERIALIZE_PAYLAOD());
	if (rc)
		goto done;

	/* copy output */
	memcpy(presults, presults_ipp, sizeof(*presults));

	rc = 0;
done:

	return rc;
}

/**
 * @brief  IPP Wrapper call for histogram ambient dmax calc
 *
 * The target reflectance in percent for the DMAX calculation
 * is set by target_reflectance input
 *
 * The fixed point format is 7.2
 *
 * @param[in]    Dev                : Device handle
 * @param[in]    target_reflectance : target reflectance to report ambient DMAX
 *                                    Percentage in 7.2 fixed point format
 * @param[in]    pdmax_cal          : DMAX calibration data
 * @param[in]    pdmax_cfg          : DMAX configuration data
 * @param[in]    pbins              : Input histogram raw bin data
 * @param[out]   pambient_dmax_mm   : Output ambient DMAX distance in [mm]
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_ipp_hist_ambient_dmax(
	VL53L1_DEV                         dev,
	uint16_t                           target_reflectance,
	VL53L1_dmax_calibration_data_t    *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
	VL53L1_histogram_bin_data_t       *pbins,
	int16_t                           *pambient_dmax_mm)
{
	struct stmvl53l1_data *data = (struct stmvl53l1_data *)
		container_of(dev, struct stmvl53l1_data, stdev);
	struct ipp_work_t *pout = &data->ipp.work_out;
	struct ipp_work_t *pin = &data->ipp.work;
	int rc;
	int16_t *pambient_dmax_mm_ipp;

	IPP_SERIALIZE_VAR;

	/* setup pin */
	IPP_SERIALIZE_START(pin->data, 4);
	IPP_SET_ARG(pin->data, 0, target_reflectance);
	IPP_SET_ARG_PTR(pin->data, 1, pdmax_cal);
	IPP_SET_ARG_PTR(pin->data, 2, pdmax_cfg);
	IPP_SET_ARG_PTR(pin->data, 3, pbins);
	pin->payload = IPP_SERIALIZE_PAYLAOD();
	pin->process_no = stmvl53l1_ipp_hist_ambient_dmax;

	/* setup pout */
	IPP_SERIALIZE_START(pout->data, 1);
	IPP_OUT_ARG_PTR(pout->data, 0, pambient_dmax_mm_ipp);

	/* do ipp */
	rc = stmvl53l1_ipp_do_wrapper(data, pin, pout, IPP_SERIALIZE_PAYLAOD());
	if (rc)
		goto done;

	/* copy output */
	memcpy(pambient_dmax_mm, pambient_dmax_mm_ipp,
		sizeof(*pambient_dmax_mm));

	rc = 0;
done:

	return rc;
}

/**
 * @brief  IPP Wrapper call for xtalk calibration post processing
 *
 * @param[in]      Dev                 : Device handle
 * @param[in]      pxtalk_ranges       : Input VL53L1_xtalk_range_results_t
 *                                       Must contain 5 ranges, 4 quadrants + 1
 *                                       full FoV
 * @param[out]     pxtalk_shape        : Output normalised Cross talk  histogram
 *                                       shape
 * @param[out]     pxtalk_cal        : Output VL53L1_xtalk_calibration_results_t
 *                                     structure
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */
VL53L1_Error VL53L1_ipp_xtalk_calibration_process_data(
	VL53L1_DEV                          dev,
	VL53L1_xtalk_range_results_t       *pxtalk_ranges,
	VL53L1_xtalk_histogram_data_t      *pxtalk_shape,
	VL53L1_xtalk_calibration_results_t *pxtalk_cal)
{
	struct stmvl53l1_data *data = (struct stmvl53l1_data *)
		container_of(dev, struct stmvl53l1_data, stdev);
	struct ipp_work_t *pout = &data->ipp.work_out;
	struct ipp_work_t *pin = &data->ipp.work;
	int rc;
	VL53L1_xtalk_histogram_data_t *pxtalk_shape_ipp;
	VL53L1_xtalk_calibration_results_t *pxtalk_cal_ipp;

	IPP_SERIALIZE_VAR;

	/* setup pin */
	IPP_SERIALIZE_START(pin->data, 1);
	IPP_SET_ARG_PTR(pin->data, 0, pxtalk_ranges);
	pin->payload = IPP_SERIALIZE_PAYLAOD();
	pin->process_no = stmvl53l1_ipp_xtalk_calibration;

	/* setup pout */
	IPP_SERIALIZE_START(pout->data, 2);
	IPP_OUT_ARG_PTR(pout->data, 0, pxtalk_shape_ipp);
	IPP_OUT_ARG_PTR(pout->data, 1, pxtalk_cal_ipp);

	/* do ipp */
	rc = stmvl53l1_ipp_do_wrapper(data, pin, pout, IPP_SERIALIZE_PAYLAOD());
	if (rc)
		goto done;

	/* copy output */
	memcpy(pxtalk_shape, pxtalk_shape_ipp, sizeof(*pxtalk_shape));
	memcpy(pxtalk_cal, pxtalk_cal_ipp, sizeof(*pxtalk_cal));

	rc = 0;
done:

	return rc;
}

/**
 * @brief  IPP Wrapper call for Generating Xtalk data from dual reflectance
 * histogram data
 *
 * @param[in]  Dev                          : Device handle
 * @param[in]  pxtalk_results               : Pointer to xtalk_results structure
 *                                            containing dual reflectance
 *                                            histogram data
 * @param[in]  expected_target_distance_mm  : User input of true target distance
 * @param[in]  higher_reflectance           : User input detailing which
 *                                            histogram data 1 or 2 has the
 *                                            highest reflectance.
 * @param[out] pxtalk_avg_samples           : Pointer to output xtalk histogram
 *                                            data
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_ipp_generate_dual_reflectance_xtalk_samples(
	VL53L1_DEV                     dev,
	VL53L1_xtalk_range_results_t  *pxtalk_results,
	uint16_t                       expected_target_distance_mm,
	uint8_t                        higher_reflectance,
	VL53L1_histogram_bin_data_t    *pxtalk_avg_samples)
{
	struct stmvl53l1_data *data = (struct stmvl53l1_data *)
		container_of(dev, struct stmvl53l1_data, stdev);
	struct ipp_work_t *pout = &data->ipp.work_out;
	struct ipp_work_t *pin = &data->ipp.work;
	int rc;
	VL53L1_histogram_bin_data_t *pxtalk_avg_samples_ipp;

	IPP_SERIALIZE_VAR;

	/* setup pin */
	IPP_SERIALIZE_START(pin->data, 3);
	IPP_SET_ARG_PTR(pin->data, 0, pxtalk_results);
	IPP_SET_ARG(pin->data, 1, expected_target_distance_mm);
	IPP_SET_ARG(pin->data, 2, higher_reflectance);
	pin->payload = IPP_SERIALIZE_PAYLAOD();
	pin->process_no = stmvl53l1_ipp_generate_dual_reflectance_xtalk_samples;

	/* setup pout */
	IPP_SERIALIZE_START(pout->data, 1);
	IPP_OUT_ARG_PTR(pout->data, 0, pxtalk_avg_samples_ipp);

	/* do ipp */
	rc = stmvl53l1_ipp_do_wrapper(data, pin, pout, IPP_SERIALIZE_PAYLAOD());
	if (rc)
		goto done;

	/* copy output */
	memcpy(pxtalk_avg_samples, pxtalk_avg_samples_ipp,
		sizeof(*pxtalk_avg_samples));

	rc = 0;
done:

	return rc;
}