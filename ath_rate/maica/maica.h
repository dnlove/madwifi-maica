/*
 * Copyright (c) 20011 The Regents of the University of California
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Author: Duy Nguyen(duy@soe.ucsc.edu)
 */

/* Indranet Technologies Ltd sponsored Derek Smithies to work
 * on this code. Derek Smithies (derek@indranet.co.nz) took parts of the
 * adm module and pasted it into this code base.
 */

/*
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * All rights reserved.
 * $Id: onoe.c 3902 2009-01-14 02:36:53Z proski $
 *
 * Copyright (c) 2005 John Bicket
 * All rights reserved
 * $Id: sample.c 2161 2007-02-27 17:45:56Z proski $
 *
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_RATE_MAICA_H
#define _DEV_ATH_RATE_MAICA_H


/* per-device state */
struct maica_softc {
	struct ath_ratectrl arc; 	/* base state */

#ifdef CONFIG_SYSCTL
	struct ctl_table_header *sysctl_header;
	struct ctl_table *sysctls;
#endif
        struct ath_softc  *sc;
        struct net_device *sc_dev; 


	struct timer_list timer;	/* periodic timer */
        int close_timer_now;
};


#define ATH_SOFTC_MAICA(sc)    ((struct maica_softc *)sc->sc_rc)

struct rate_info {
	int rate;
	int rix;
	int rateCode;
	int shortPreambleRateCode;
};

/* per-node state */
struct maica_node {
        int static_rate_ndx; /*User has bypassed dynamic selection. Fix on one rate */
	int num_rates;

	int maica_nextcheck;

	struct rate_info rates[IEEE80211_RATE_MAXSIZE];

	unsigned perfect_tx_time[IEEE80211_RATE_MAXSIZE];  /* transmit time for 0 retries */
	unsigned retry_count[IEEE80211_RATE_MAXSIZE];      /*The number of retrys permitted for this particular rate */
	unsigned retry_adjusted_count[IEEE80211_RATE_MAXSIZE]; /*retry_count, but altered, depending on if this is a very poor (or very good) link */
	int current_rate;

	u_int32_t	maica_tx_ok;
	u_int32_t	maica_tx_err;
	u_int32_t	maica_tx_retr;
	int 		maica_tx_credit;

	//duy remember these are indices only
	int maica_tx_rate0;
        int maica_tx_rate1;
        int maica_tx_rate2;
        int maica_tx_rate3;


	u_int32_t		rs_rateattempts[IEEE80211_RATE_MAXSIZE];
	u_int32_t               rs_thisprob    [IEEE80211_RATE_MAXSIZE];
	u_int32_t		rs_ratesuccess [IEEE80211_RATE_MAXSIZE];

	u_int32_t		rs_lastrateattempts[IEEE80211_RATE_MAXSIZE];
	u_int32_t		rs_lastratesuccess [IEEE80211_RATE_MAXSIZE];
	u_int32_t               rs_probability [IEEE80211_RATE_MAXSIZE]; /* units of parts per thousand */
	u_int64_t               rs_succ_hist   [IEEE80211_RATE_MAXSIZE]; 
	u_int64_t               rs_att_hist    [IEEE80211_RATE_MAXSIZE]; 
       
	u_int32_t               rs_this_tp     [IEEE80211_RATE_MAXSIZE]; /*Throughput, each rate */

	/** These four parameters are indexes into the current rate
	    table, and are calculated in ath_rate_statistics(), which
	    happens every time the timer for rate adjustment fires */
	int max_tp_rate;       /*  Has the current highest recorded throughput */
	int max_tp_rate2;      /*  Has the second highest recorded throughput */
	int max_tp_rate3;      /*  Has the 3rd highest recorded throughput */
	int max_tp_rate4;      /*  Has the 4th highest recorded throughput */
	int max_prob_rate;     /*  This rate has the highest chance of success. */

	


	/**These two variables are used to keep track of what
	   percentage of packets have been used to do sample on. 
	   Thus,if ath_lookaround_rate is set to 10%, we can expect that
	   sample_count
           ------------                    = 0.1
	   sample_count + packet_count                           */
	int packet_count;     /*  The number of times we have  
				  sent a packet to this node. */

       /**Random number generator is
           Rn+1 = (A*Rn) + B. 

           This Random number generator determines when we send a maica
           packet, or a packet at an optimal rate.*/
        int random_n;
        int a, b;          /**Coefficients of the random thing */
	
};


#define	ATH_NODE_MAICA(an)	((struct maica_node *)&an[1])


#ifndef MIN
#define MIN(a,b)        ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b)        ((a) > (b) ? (a) : (b))
#endif

#if 0
#define WIFI_CW_MIN 31
#define WIFI_CW_MAX 1023
#else

#define WIFI_CW_MIN 3
#define WIFI_CW_MAX 10
#endif

struct ar5212_desc {
	/*
	 * tx_control_0
	 */
	u_int32_t frame_len:12;
	u_int32_t reserved_12_15:4;
	u_int32_t xmit_power:6;
	u_int32_t rts_cts_enable:1;
	u_int32_t veol:1;
	u_int32_t clear_dest_mask:1;
	u_int32_t ant_mode_xmit:4;
	u_int32_t inter_req:1;
	u_int32_t encrypt_key_valid:1;
	u_int32_t cts_enable:1;

	/*
	 * tx_control_1
	 */
	u_int32_t buf_len:12;
	u_int32_t more:1;
	u_int32_t encrypt_key_index:7;
	u_int32_t frame_type:4;
	u_int32_t no_ack:1;
	u_int32_t comp_proc:2;
	u_int32_t comp_iv_len:2;
	u_int32_t comp_icv_len:2;
	u_int32_t reserved_31:1;

	/*
	 * tx_control_2
	 */
	u_int32_t rts_duration:15;
	u_int32_t duration_update_enable:1;
	u_int32_t xmit_tries0:4;
	u_int32_t xmit_tries1:4;
	u_int32_t xmit_tries2:4;
	u_int32_t xmit_tries3:4;

	/*
	 * tx_control_3
	 */
	u_int32_t xmit_rate0:5;
	u_int32_t xmit_rate1:5;
	u_int32_t xmit_rate2:5;
	u_int32_t xmit_rate3:5;
	u_int32_t rts_cts_rate:5;
	u_int32_t reserved_25_31:7;

	/*
	 * tx_status_0
	 */
	u_int32_t frame_xmit_ok:1;
	u_int32_t excessive_retries:1;
	u_int32_t fifo_underrun:1;
	u_int32_t filtered:1;
	u_int32_t rts_fail_count:4;
	u_int32_t data_fail_count:4;
	u_int32_t virt_coll_count:4;
	u_int32_t send_timestamp:16;

	/*
	 * tx_status_1
	 */
	u_int32_t done:1;
	u_int32_t seq_num:12;
	u_int32_t ack_sig_strength:8;
	u_int32_t final_ts_index:2;
	u_int32_t comp_success:1;
	u_int32_t xmit_antenna:1;
	u_int32_t reserved_25_31_x:7;
} __packed;

#endif /* _DEV_ATH_RATE_MAICA_H */

/* The comment below is magic for those who use emacs to edit this file. */
/* With the comment below, the tab key does auto indent to 8 spaces.     */

/*
 * Local Variables:
 * mode:c
 * c-file-style:"linux"
 * c-basic-offset:8
 * End:
 */
