/*
 * Copyright (c) 2011 The Regents of the University of California 
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
 * $Id: minstrel.c  
 */

/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * All rights reserved.
 * $Id: onoe.c 3902 2009-01-14 02:36:53Z proski $
 *
 * Copyright (c) 2005 John Bicket
 * All rights reserved
 * $Id: sample.c 2161 2007-02-27 17:45:56Z proski $
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


#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/net.h>		/* for net_random */
#include <linux/vmalloc.h>

#include <asm/uaccess.h>

#include <net80211/if_media.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_rate.h>

#include "if_athvar.h"
#include "ah_desc.h"

#include "maica.h"

#define MAICA_DEBUG

#ifdef MAICA_DEBUG
enum {
	ATH_DEBUG_RATE		= 0x00000010	/* rate control */
};
#define DPRINTF(sc, _fmt, ...) do {		\
	if (sc->sc_debug & ATH_DEBUG_RATE)	\
		printk(_fmt, __VA_ARGS__);	\
} while(0)
#else
#define	DPRINTF(sc, _fmt, ...)
#endif

#define ONE_SECOND (1000 * 1000)  /* 1 second, or 1000 milliseconds, eternity in other words */

#include "release.h"

static char *version = "1.0 (" RELEASE_VERSION ")";
static char *dev_info = "ath_rate_maica";

#define STALE_FAILURE_TIMEOUT_MS 10000
#define ENABLE_MRR 1

static int ath_timer_interval = 100; /* every 1/10 second, timer runs */

static void ath_timer_function(unsigned long data);

static int ath_rate_raise = 10;

static int ath_ewma_level      = 75;
static int ath_segment_size    = 6000;


static void ath_rate_ctl_reset(struct ath_softc *, struct ieee80211_node *);

/* Calculate the throughput and probability of success for each node
 * we are talking on, based on the statistics collected during the
 * last timer period. */
static void ath_rate_statistics(void *arg, struct ieee80211_node *ni);
static void ath_rate_update(struct ath_softc *, struct ieee80211_node *, int);
static void check_rate (struct ath_softc *, struct ieee80211_node *ni);


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ath_ewma_level, "i");
MODULE_PARM(ath_segment_size, "i");
#else
#include <linux/moduleparam.h>
module_param(ath_ewma_level, 		int, 0600);
module_param(ath_segment_size, 		int, 0600);
#endif
MODULE_PARM_DESC(ath_ewma_level, " scaling % used in ewma rolloff calculations  (75) ");
MODULE_PARM_DESC(ath_segment_size, " max duration of time to spend in either of the first two mrr segments (6000)");


/* convert rate to regular index */
static __inline int 
rate_to_ndx(struct maica_node *sn, int rate)
{
	unsigned int x = 0;
	for (x = 0; x < sn->num_rates; x++)
		if (sn->rates[x].rate == rate)
			return x;
	return -1;
}

/* Calculate the transmit duration of a frame. */
static unsigned
calc_usecs_unicast_packet(struct ath_softc *sc, int length, 
	int rix, int short_retries, int long_retries)
{
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	struct ieee80211com *ic = &sc->sc_ic;
	unsigned t_slot = 20;
	unsigned t_difs = 50; 
	unsigned t_sifs = 10; 
	unsigned int x = 0, tt = 0;
	unsigned int cix = rt->info[rix].controlRate;  
	int rts =0, cts = 0;
	int cw = WIFI_CW_MIN;

	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	if (!rt->info[rix].rateKbps) {
		printk(KERN_WARNING "rix %d (%d) bad ratekbps %d mode %u\n",
		       rix, rt->info[rix].dot11Rate,
		       rt->info[rix].rateKbps,
		       sc->sc_curmode);
		return 0;
	}
	
	/* XXX getting MAC/PHY level timings should be fixed for turbo
	 * rates, and there is probably a way to get this from the
	 * HAL... */
	switch (rt->info[rix].phy) {
	case IEEE80211_T_OFDM:
		t_slot = 9;
		t_sifs = 16;
		t_difs = 28;
		/* fall through */
		/* XXX: WTF? */
	case IEEE80211_T_TURBO:
		t_slot = 9;
		t_sifs = 8;
		t_difs = 28;
		break;
	case IEEE80211_T_DS:
		/* fall through to default */
	default:
		/* pg 205 ieee.802.11.pdf */
		t_slot = 20;
		t_difs = 50;
		t_sifs = 10;
	}

	rts = cts = 0;

	//ddn check for protection mode
	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->info[rix].phy == IEEE80211_T_OFDM) {
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			rts = 1;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			cts = 1;

		//ddn geting the control index that is 
		//associated with this protection mode
		cix = rt->info[sc->sc_protrix].controlRate;
	}

#if 0
	if (length > ic->ic_rtsthreshold)
		rts = 1;
#endif

	if (rts || cts) {
		int ctsrate = rt->info[cix].rateCode;
		int ctsduration = 0;

		if (!rt->info[cix].rateKbps) {
#if 0
			printk(KERN_WARNING "cix %d (%d) bad ratekbps %d mode %u\n",
			       cix, rt->info[cix].dot11Rate,
			       rt->info[cix].rateKbps,
			       sc->sc_curmode);
#endif
			return 0;
		}
		

		ctsrate |= rt->info[cix].shortPreamble;
		if (rts)	/* SIFS + CTS */
			ctsduration += rt->info[cix].spAckDuration;

		ctsduration += ath_hal_computetxtime(sc->sc_ah,
						     rt, length, rix, AH_TRUE);

		if (cts)	/* SIFS + ACK */
			ctsduration += rt->info[cix].spAckDuration;

		tt += (short_retries + 1) * ctsduration;
	}
	tt += t_difs;
	tt += (long_retries + 1) * (t_sifs + rt->info[rix].spAckDuration);
	tt += (long_retries + 1) * ath_hal_computetxtime(sc->sc_ah, rt, length, 
						rix, AH_TRUE);
	for (x = 0; x <= short_retries + long_retries; x++) {
		cw = MIN(WIFI_CW_MAX, (cw + 1) * 2);
		tt += (t_slot * cw / 2);
	}
	return tt;
}

static void
ath_rate_node_init(struct ath_softc *sc, struct ath_node *an)
{
	/* NB: assumed to be zero'd by caller */
	ath_rate_ctl_reset(sc, &an->an_node);
}


static void
ath_rate_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
}

#if 0
static void
ath_rate_node_copy(struct ath_softc *sc,
	struct ath_node *dst, const struct ath_node *src)
{
	struct maica_node *odst = ATH_NODE_MAICA(dst);
	const struct maica_node *osrc = (const struct maica_node *)&src[1];
	memcpy(odst, osrc, sizeof(struct maica_node));
}
#endif

static void
ath_rate_findrate(struct ath_softc *sc, struct ath_node *an,
        int shortPreamble, size_t frameLen,
        u_int8_t *rix, int *try0, u_int8_t *txrate)
{
        struct maica_node *sn = ATH_NODE_MAICA(an);
        struct ieee80211com *ic = &sc->sc_ic;

        unsigned int ndx ;
	int mrr;

        if (sn->num_rates <= 0) {
                printk(KERN_WARNING "%s: no rates for %s?\n",
                       dev_info,
                       ether_sprintf(an->an_node.ni_macaddr));
                return;
        }
  
        mrr = sc->sc_mrretry && !(ic->ic_flags & IEEE80211_F_USEPROT) && ENABLE_MRR;

        if (sn->static_rate_ndx != -1) {
                ndx = sn->static_rate_ndx;
        } else {
		sn->packet_count++;
	 	ndx = sn->maica_tx_rate0;
	}

        if ((sn->static_rate_ndx != -1) || !mrr) 
                *try0 = ATH_TXMAXTRY;
	else
                *try0 = sn->retry_adjusted_count[ndx];

        KASSERT(ndx >= 0 && ndx < sn->num_rates,
                ("%s: bad ndx (%d/%d) for %s?\n",
                 dev_info, ndx, sn->num_rates,
                 ether_sprintf(an->an_node.ni_macaddr)));

        *rix = sn->rates[ndx].rix;
        if (shortPreamble)
                *txrate = sn->rates[ndx].shortPreambleRateCode;
        else
                *txrate = sn->rates[ndx].rateCode;
}

//ddn for multi-rate retry
static void
ath_rate_setupxtxdesc(struct ath_softc *sc, struct ath_node *an,
	struct ath_desc *ds, int shortPreamble, size_t frame_size, u_int8_t rix)
{
	struct maica_node *sn = ATH_NODE_MAICA(an);
	int rc1, rc2, rc3;         /* Index into the rate table, so for example, it is  0..11 */
	int rixc1, rixc2, rixc3;   /* The actual bit rate used */

	if (sn->num_rates <= 0) {
		DPRINTF(sc, "%s: no rates for %s\n", dev_info,
			ether_sprintf(an->an_node.ni_macaddr));
		return;
	}
	//ddn may need to set i up different
	rc1 = sn->maica_tx_rate1;  
	rc2 = sn->maica_tx_rate2;
	rc3 = sn->maica_tx_rate3;


        KASSERT(rc1 >= 0 && rc1 < sn->num_rates,
                ("%s: bad rc1 (%d/%d) for %s?\n",
                 dev_info, rc1, sn->num_rates,
                 ether_sprintf(an->an_node.ni_macaddr)));

        KASSERT(rc2 >= 0 && rc2 < sn->num_rates,
                ("%s: bad rc2 (%d/%d) for %s?\n",
                 dev_info, rc2, sn->num_rates,
                 ether_sprintf(an->an_node.ni_macaddr)));

        KASSERT(rc3 >= 0 && rc3 < sn->num_rates,
                ("%s: bad rc3 (%d/%d) for %s?\n",
                 dev_info, rc3, sn->num_rates,
                 ether_sprintf(an->an_node.ni_macaddr)));

	if (shortPreamble) {
		rixc1 = sn->rates[rc1].shortPreambleRateCode;
		rixc2 = sn->rates[rc2].shortPreambleRateCode;
		rixc3 = sn->rates[rc3].shortPreambleRateCode;
	} else {
		rixc1 = sn->rates[rc1].rateCode;
		rixc2 = sn->rates[rc2].rateCode;
		rixc3 = sn->rates[rc3].rateCode;
	}

	ath_hal_setupxtxdesc(sc->sc_ah, ds,
			     rixc1, sn->retry_adjusted_count[rc1],   /* series 1 */
			     rixc2, sn->retry_adjusted_count[rc2],   /* series 2 */
			     rixc3, sn->retry_adjusted_count[rc3]    /* series 3 */
			     );
}

static void
ath_rate_tx_complete(struct ath_softc *sc,
	struct ath_node *an, const struct ath_desc *ds)
{
	struct maica_node *sn = ATH_NODE_MAICA(an);
	struct ieee80211com *ic = &sc->sc_ic;
	const struct ar5212_desc *ads = (const struct ar5212_desc *)&ds->ds_ctl0;
	int final_rate = 0;
	int tries = 0;
	int ndx = -1;
	int mrr;
	int final_ndx;
	int rate0, tries0, ndx0;
	int rate1, tries1, ndx1;
	int rate2, tries2, ndx2;
	int rate3, tries3, ndx3;
	
	/* This is the index in the retry chain we finish at.
	 * With no retransmits, it is always 0.
	 * int finalTSIdx = ads->final_ts_index; */
	final_rate = sc->sc_hwmap[ds->ds_txstat.ts_rate &~ HAL_TXSTAT_ALTRATE].ieeerate;
	final_ndx = rate_to_ndx(sn, final_rate);
	if (final_ndx >= sn->num_rates) {
		DPRINTF(sc,"%s: final ndx too high\n", __func__);
		final_ndx = 0;
	}
	if (final_ndx < 0) {
		DPRINTF(sc, "%s: final ndx too low\n", __func__);
		final_ndx = 0;
	}		

	/* tries is the total number of times we have endeavoured to
	 * send this packet, and is a sum of the #attempts at each
	 * level in the multi rate retry chain */
	tries = ds->ds_txstat.ts_shortretry + ds->ds_txstat.ts_longretry + 1;
	
	//ddn
	sn->maica_tx_retr = ds->ds_txstat.ts_shortretry + ds->ds_txstat.ts_longretry;

	if (sn->num_rates <= 0) {
		DPRINTF(sc, "%s: %s %s no rates yet\n", dev_info, 
			ether_sprintf(an->an_node.ni_macaddr), __func__);
		return;
	}

	if (!ds->ds_txstat.ts_status){  /* Success when sending a packet*/
		sn->rs_ratesuccess[final_ndx]++;
		
		sn->maica_tx_ok++;
	}
	else {

		sn->maica_tx_err++;

	}

	mrr = sc->sc_mrretry && !(ic->ic_flags & IEEE80211_F_USEPROT) && ENABLE_MRR;

	if (!mrr) {
		if (ndx >= 0 && ndx < sn->num_rates) {
			sn->rs_rateattempts[ndx]++; /* only one rate was used */
		}
		return;
	}

	//ddn the relationship between tries and tries1, tries2, tries3
	//note tries is the total retries for this packet
	//tries1 total retries for rate chain 1
	//tries2 total retries for rate chain 2 and so on
	

	/* Now, query the hal/hardware to find out the contents of the multirate retry chain.
	 * If we have it set to 6,3,2,2, this call will always return 6,3,2,2. For some packets, we can
	 * get a mrr of 0, -1, -1, -1, which indicates there is no chain installed for that packet */
	rate0 = sc->sc_hwmap[ads->xmit_rate0].ieeerate;
	tries0 = ads->xmit_tries0;
	ndx0 = rate_to_ndx(sn, rate0);
	
	rate1 = sc->sc_hwmap[ads->xmit_rate1].ieeerate;
	tries1 = ads->xmit_tries1;
	ndx1 = rate_to_ndx(sn, rate1);
	
	rate2 = sc->sc_hwmap[ads->xmit_rate2].ieeerate;
	tries2 = ads->xmit_tries2;
	ndx2 = rate_to_ndx(sn, rate2);
	
	rate3 = sc->sc_hwmap[ads->xmit_rate3].ieeerate;
	tries3 = ads->xmit_tries3;
	ndx3 = rate_to_ndx(sn, rate3);

	sn->rs_rateattempts[ndx0] += MIN(tries, tries0);
	if (tries <= tries0)
		return;

	if (tries1 < 0)
		return;
	tries = tries - tries0;
	sn->rs_rateattempts[ndx1] += MIN(tries, tries1);
	if (tries <= tries1)
		return; 

	if  (tries2 < 0)
		return;
	tries = tries - tries1;
	sn->rs_rateattempts[ndx2] += MIN(tries, tries2);
	if (tries <= tries2)
		return;

	if  (tries3 < 0)
		return;
	tries = tries - tries2;
	sn->rs_rateattempts[ndx3] += MIN(tries, tries3);	


}

static void
ath_rate_newassoc(struct ath_softc *sc, struct ath_node *an, int isnew)
{
	DPRINTF(sc, "%s: %s %s\n", dev_info,
		ether_sprintf(an->an_node.ni_macaddr), __func__);
	if (isnew)
		ath_rate_ctl_reset(sc, &an->an_node);
}

/* Initialize the tables for a node. */
static void
ath_rate_ctl_reset(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ath_node *an = ATH_NODE(ni);
	struct maica_node *sn = ATH_NODE_MAICA(an);
	struct ieee80211vap *vap = ni->ni_vap;
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	unsigned int x;
	int retry_index, tx_time;
	int srate;
	int ndx = 0;

	sn->num_rates = 0;
	sn->max_tp_rate = 0;
	sn->max_tp_rate2 = 0;
	sn->max_tp_rate3 = 0;
	sn->max_tp_rate4 = 0;
	sn->max_prob_rate = 0;
        sn->packet_count = 0;


	sn->maica_tx_rate1 = 0;
        sn->maica_tx_rate2 = 0;
        sn->maica_tx_rate3 = 0;

	sn->maica_tx_ok =0;
	sn->maica_tx_err =0;
	sn->maica_tx_retr =0;
	sn->maica_tx_credit =0;





	if (rt == NULL) {
		DPRINTF(sc, "no rates yet! mode %u\n", sc->sc_curmode);
		return;
	}
        sn->static_rate_ndx = -1;

	//ddn ni= negotiated rate from ieee80211
	sn->num_rates = ni->ni_rates.rs_nrates;
        for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
		if (sn->rates[x].rix == 0xff) {
			DPRINTF(sc, "%s: %s ignore bogus rix at %d\n",
					dev_info, __func__, x);
			continue;
		}
		sn->rs_rateattempts 	[x] = 0;
		sn->rs_thisprob 	[x] = 0;
		sn->rs_ratesuccess 	[x] = 0;
		sn->rs_lastrateattempts [x] = 0;
		sn->rs_lastratesuccess	[x] = 0;
		sn->rs_probability	[x] = 0;
		sn->rs_succ_hist	[x] = 0;
		sn->rs_att_hist 	[x] = 0;
		sn->rs_this_tp 		[x] = 0;

		sn->rates[x].rate = ni->ni_rates.rs_rates[x] & IEEE80211_RATE_VAL;
		sn->rates[x].rix = sc->sc_rixmap[sn->rates[x].rate];
		if (sn->rates[x].rix == 0xff) {
			DPRINTF(sc, "%s: %s ignore bogus rix at %d\n",
				dev_info, __func__, x);
			continue;
		}
		sn->rates[x].rateCode = rt->info[sn->rates[x].rix].rateCode;
		sn->rates[x].shortPreambleRateCode = 
			rt->info[sn->rates[x].rix].rateCode | 
			rt->info[sn->rates[x].rix].shortPreamble;
	}

	ni->ni_txrate = 0;
	sn->num_rates = ni->ni_rates.rs_nrates;

	if (sn->num_rates <= 0) {
		DPRINTF(sc, "%s: %s %s no rates (fixed %d) \n",
			dev_info, __func__, ether_sprintf(ni->ni_macaddr),
			vap->iv_fixed_rate);
		/* there are no rates yet we're done */
		return;
	}

	if (vap->iv_fixed_rate != IEEE80211_FIXED_RATE_NONE) {
		srate = sn->num_rates - 1;

		/* A fixed rate is to be used; ic_fixed_rate is an
		 * index into the supported rate set.  Convert this
		 * to the index into the negotiated rate set for
		 * the node.  We know the rate is there because the
		 * rate set is checked when the station associates. */
		/* NB: the rate set is assumed sorted */
		for (; srate >= 0 && (ni->ni_rates.rs_rates[srate] & IEEE80211_RATE_VAL) != vap->iv_fixed_rate; srate--);

		KASSERT(srate >= 0,
			("fixed rate %d not in rate set", vap->iv_fixed_rate));

		sn->static_rate_ndx = srate;
		ni->ni_txrate = srate;
		DPRINTF(sc, "%s: %s %s fixed rate %d%sMbps\n",
			dev_info, __func__, ether_sprintf(ni->ni_macaddr), 
			sn->rates[srate].rate / 2,
			(sn->rates[srate].rate % 0x1) ? ".5" : " ");
		return;
	}
	
	for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
		sn->rs_rateattempts	[x] = 0;
		sn->rs_thisprob		[x] = 0;
		sn->rs_ratesuccess 	[x] = 0;
		sn->rs_probability 	[x] = 0;
		sn->rs_lastrateattempts [x] = 0;
		sn->rs_lastratesuccess 	[x] = 0;
		sn->rs_succ_hist 	[x] = 0;
		sn->rs_att_hist 	[x] = 0;
		sn->perfect_tx_time 	[x] = 
			calc_usecs_unicast_packet(sc, 1200, 
						  sn->rates[x].rix,
						  0, 0);
		sn->retry_count 	[x] = 1;
		sn->retry_adjusted_count[x] = 1;

		for (retry_index = 2; retry_index < 11; retry_index++) {
			tx_time = calc_usecs_unicast_packet(sc, 1200, sn->rates[x].rix, 0, retry_index);
			if (tx_time >  ath_segment_size) 
				break;
			sn->retry_count[x] = retry_index;
			sn->retry_adjusted_count[x] = retry_index;
		}
	}

#if 0
	DPRINTF(sc, "%s: Retry table for this node\n", __func__);
	      for (x = 0; x < ni->ni_rates.rs_nrates; x++) 
		     DPRINTF(sc, "%2d  %2d %6d  \n",x, sn->retry_count[x], sn->perfect_tx_time[x]);
#endif



	//ddn set the initial rate half way
	/* set the initial rate */
	for (ndx = sn->num_rates-1; ndx > 0; ndx--)
		if (sn->rates[ndx].rate <= 72)
			break;
	sn->current_rate = ndx;
	
	//initial rate is half way
	ath_rate_update(sc,ni, ndx); 

}


static void
ath_rate_cb(void *arg, struct ieee80211_node *ni)
{
        ath_rate_ctl_reset(netdev_priv(ni->ni_ic->ic_dev), ni);
}

/* Reset the rate control state for each 802.11 state transition. */
static void
ath_rate_newstate(struct ieee80211vap *vap, enum ieee80211_state newstate)
{
	struct ieee80211com *ic = vap->iv_ic;
	
	if (newstate == IEEE80211_S_RUN) {
		if (ic->ic_opmode != IEEE80211_M_STA) {
			/* Sync rates for associated stations and neighbors. */
			ieee80211_iterate_nodes(&ic->ic_sta, ath_rate_cb, NULL);
		}
		ath_rate_newassoc(netdev_priv(ic->ic_dev), ATH_NODE(vap->iv_bss), 1);
	}
}

static void
ath_timer_function(unsigned long data)
{
	struct maica_softc *ssc = (struct maica_softc *) data;
	struct ath_softc *sc = ssc->sc;
	struct ieee80211com *ic;
	struct net_device *dev = ssc->sc_dev;
	struct timer_list *timer;
	unsigned int interval = ath_timer_interval;

	if (dev == NULL) 
		DPRINTF(sc, "%s: 'dev' is null in this timer \n", __func__);

	if (sc == NULL) 
		DPRINTF(sc, "%s: 'sc' is null in this timer\n", __func__);

	ic = &sc->sc_ic;

	if (ssc->close_timer_now)
		return;
	
	if (dev->flags & IFF_RUNNING) {
		sc->sc_stats.ast_rate_calls++;

		if (ic->ic_opmode == IEEE80211_M_STA) {
			struct ieee80211vap *tmpvap;
			TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
				ath_rate_statistics(sc, tmpvap->iv_bss);/* NB: no reference */
			}
		} else
                        ieee80211_iterate_nodes(&ic->ic_sta, ath_rate_statistics, sc);
	}

	if (ic->ic_opmode == IEEE80211_M_STA)
		interval = ath_timer_interval >> 1;

	timer  = &(ssc->timer);
	if (timer == NULL) 
		DPRINTF(sc, "%s: timer is null - leave it\n", __func__);

        timer->expires = jiffies + ((HZ * interval) / 1000);
        add_timer(timer);
}

static void
ath_rate_statistics(void *arg, struct ieee80211_node *ni)
{
//	struct ath_softc *sc = arg;

        struct ath_node *an = (struct ath_node *) ni;
        struct ieee80211_rateset *rs = &ni->ni_rates;
        struct maica_node *sn = ATH_NODE_MAICA(an);
        unsigned int i;
        u_int32_t p;
	u_int32_t micro_secs;
	u_int32_t max_prob, index_max_prob;
	u_int32_t max_tp, index_max_tp, index_max_tp2, index_max_tp3, index_max_tp4;

	/* Calculate statistics for each date rate in the table */
	/* micro_secs is the time to transmit 1200 bytes, or 9600 bits.*/
        for (i = 0; i < rs->rs_nrates; i++) {
		micro_secs = sn->perfect_tx_time[i];
		if (micro_secs == 0) 
			micro_secs = ONE_SECOND;
		
                if (sn->rs_rateattempts[i] != 0) {
                        p = (sn->rs_ratesuccess[i] * 18000) / sn->rs_rateattempts[i];
                        sn->rs_succ_hist[i] += sn->rs_ratesuccess[i];
                        sn->rs_att_hist[i]  += sn->rs_rateattempts[i];
			sn->rs_thisprob[i] = p;
			p = ((p * (100 - ath_ewma_level)) + (sn->rs_probability[i] * ath_ewma_level))/100;
			sn->rs_probability[i] = p;
			sn->rs_this_tp[i] = p * (ONE_SECOND/micro_secs);
			sn->rs_lastratesuccess[i] = sn->rs_ratesuccess[i];
			sn->rs_lastrateattempts[i] = sn->rs_rateattempts[i];
                        sn->rs_ratesuccess[i] = 0;
                        sn->rs_rateattempts[i] = 0;
		} else {
			sn->rs_lastratesuccess[i] = 0;
			sn->rs_lastrateattempts[i] = 0;
		}

		/* Sample less often below the 10% chance of success.
		 * Sample less often above the 95% chance of success.
		 * sn->rs_probability is in units of 0..18000(100%), which avoids rounding issues.*/
		if ((sn->rs_probability[i] > 17100) || ( sn->rs_probability[i] < 1800)) {
			sn->retry_adjusted_count[i] = sn->retry_count[i]>> 1;
			if (sn->retry_adjusted_count[i] > 2)
				sn->retry_adjusted_count[i] = 2;
		} else 
			sn->retry_adjusted_count[i] = sn->retry_count[i];
		if (sn->retry_adjusted_count[i] == 0)
			sn->retry_adjusted_count[i] = 1;
	}
         
	/* The High speed rates (e.g 54mbps) is checked last. If
	 * throughput is the same for two rates, we prefer the
	 * lower rate, as this has a better chance of success. */
	max_prob = 0; 
	index_max_prob = 0;
	max_tp = 0; 
	index_max_tp  = 0;
	index_max_tp2 = 0;
	index_max_tp3 = 0;
	index_max_tp4 = 0;
	
	/* This code could have been moved up into the previous
	 * loop. More readable to have it here */
	for (i = 0; i < rs->rs_nrates; i++) {
		if (max_tp < sn->rs_this_tp[i]) {
			index_max_tp = i;
			max_tp = sn->rs_this_tp[i];
		}

		if (max_prob < sn->rs_probability[i]) {
			index_max_prob = i;
			max_prob = sn->rs_probability[i];
		}
	}
         
	max_tp = 0;
	for (i = 0; i < rs->rs_nrates; i++) {
		if ((i != index_max_tp) && (max_tp < sn->rs_this_tp[i])) {
			index_max_tp2 = i;
			max_tp = sn->rs_this_tp[i];
		}
	}
	max_tp = 0;
	for (i = 0; i < rs->rs_nrates; i++) {
		if ((i != index_max_tp && i != index_max_tp2) && (max_tp < sn->rs_this_tp[i])) {
			index_max_tp3 = i;
			max_tp = sn->rs_this_tp[i];
		}
	}
	max_tp = 0;
	for (i = 0; i < rs->rs_nrates; i++) {
		if ((i != index_max_tp && i != index_max_tp2 && i != index_max_tp3) && (max_tp < sn->rs_this_tp[i])) {
			index_max_tp4 = i;
			max_tp = sn->rs_this_tp[i];
		}
	}


	sn->max_tp_rate   = index_max_tp;
	sn->max_tp_rate2  = index_max_tp2;
	sn->max_tp_rate3  = index_max_tp3;
	sn->max_tp_rate4  = index_max_tp4;
	sn->max_prob_rate = index_max_prob;
	sn->current_rate  = index_max_tp;

	check_rate(arg, ni);

}
static void
check_rate (struct ath_softc *sc, struct ieee80211_node *ni) {

	int nrate, enough=0;
        struct ieee80211_rateset *rs = &ni->ni_rates;
        u_int32_t error_thres; 

	struct ath_node *an = ATH_NODE(ni);
	struct maica_node *sn = ATH_NODE_MAICA(an);

        error_thres = sn->maica_tx_ok * 3/10;

	enough = (sn->maica_tx_ok + sn->maica_tx_err >= ath_rate_raise);

	nrate = ni->ni_txrate;

	if (enough &&  sn->maica_tx_ok == 0 && sn->maica_tx_err > 0 ){
		if( nrate > 0 ) 
			nrate--;
		sn->maica_tx_credit = 0;
#ifdef MAICA_DEBUG
		printk(KERN_DEBUG "decrease local rate to%2d because tx_err > 0 or tx_ok=0\n", nrate);
#endif

	}

	if (enough && (sn->maica_tx_ok < sn->maica_tx_retr ) ){
		if( nrate > 0 ) 
			nrate--;
		sn->maica_tx_credit = 0;
#ifdef MAICA_DEBUG
		printk(KERN_DEBUG "decrease local rate to%2d because tx_ok %d < tx_retr %d \n", nrate, sn->maica_tx_ok, sn->maica_tx_retr);
#endif

	}
 	if (enough && sn->maica_tx_err >= error_thres) {
		if( nrate > 0 ) 
			nrate--;
		
#ifdef MAICA_DEBUG
		printk(KERN_DEBUG "decrease local rate to%2d because tx_err > error_thres\n", nrate);
#endif
		//ddn multiplicative decrease
        	if (enough && sn->maica_tx_err > sn->maica_tx_ok) {
			if (nrate >= 4)
				nrate = nrate * 3/4;
#ifdef MAICA_DEBUG
		printk(KERN_DEBUG "multiplicative decrease local rate to%2d because tx_err > tx_ok \n", nrate);
#endif
		}
		sn->maica_tx_credit = 0;
	}
	if (enough && sn->maica_tx_err < error_thres) 
		sn->maica_tx_credit++;

	if(sn->maica_tx_credit >= ath_rate_raise) {
		sn->maica_tx_credit = 0;

		if(nrate + 1 < rs->rs_nrates) 
			nrate++;
	}


	if(nrate != ni->ni_txrate) {
#ifdef MAICA_DEBUG
		printk(KERN_DEBUG " set ni_txrate %d to nrate %d\n", ni->ni_txrate, nrate);
#endif
		ath_rate_update(sc, ni, nrate);

	} else if(enough) {
#ifdef MAICA_DEBUG
//	printk(KERN_DEBUG "enough %s: ok %d: err %d retr %d\n ", 
//	ether_sprintf(ni->ni_macaddr), sn->maica_tx_ok, sn->maica_tx_err,
//	sn->maica_tx_retr );
#endif
		sn->maica_tx_ok = sn->maica_tx_err = sn->maica_tx_retr = 0;
	}

}


static void
ath_rate_update (struct ath_softc *sc, struct ieee80211_node *ni, int rate)
{
	struct ath_node *an = ATH_NODE(ni);
	struct maica_node *sn = ATH_NODE_MAICA(an);

	struct ieee80211com *ic = &sc->sc_ic;
	
	const HAL_RATE_TABLE *rt = sc->sc_currates;

	int mrr;


	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	ni->ni_txrate = rate;

	if(ni->ni_rates.rs_nrates == 0)
		goto done;
	
	sn->maica_tx_rate0 = rate;

	mrr = sc->sc_mrretry && !(ic->ic_flags & IEEE80211_F_USEPROT) && ENABLE_MRR;

	if(mrr) {
		//first retry
		if(--rate >=0)
			sn->maica_tx_rate1 = rate;
		else
			sn->maica_tx_rate1 = 0;

		if( rate > 2)
			rate = (int) rate /2;
		else
			--rate;

		//second retry
		if( rate >= 0 )
			sn->maica_tx_rate2 = rate; 
		else 
			sn->maica_tx_rate2 = 0;

		//third retry	
		sn->maica_tx_rate3 = 0;
	} else {
		sn->maica_tx_rate1 = 0;
		sn->maica_tx_rate2 = 0;
		sn->maica_tx_rate3 = 0;
	}
done:
	sn->maica_tx_ok = sn->maica_tx_err = sn->maica_tx_retr = sn->maica_tx_credit = 0;

}

static struct ath_ratectrl *
ath_rate_attach(struct ath_softc *sc)
{
	struct maica_softc *osc;
	DPRINTF(sc, "%s: %s\n", dev_info, __func__);
	
        _MOD_INC_USE(THIS_MODULE, return NULL);
	osc = kmalloc(sizeof(struct maica_softc), GFP_ATOMIC);
	if (osc == NULL) {
                _MOD_DEC_USE(THIS_MODULE);
		return NULL;
	}

	osc->arc.arc_space = sizeof(struct maica_node);
	osc->arc.arc_vap_space = 0;

	osc->close_timer_now = 0;
	init_timer(&osc->timer);
 	osc->sc          = sc;
	osc->sc_dev      = sc->sc_dev;
	osc->timer.function = ath_timer_function;
	osc->timer.data = (unsigned long)osc;
	
        osc->timer.expires = jiffies + HZ;
	add_timer(&osc->timer);
	
	return &osc->arc;
}

static void 
ath_rate_detach(struct ath_ratectrl *arc)
{
 	struct maica_softc *osc = (struct maica_softc *) arc;
	osc->close_timer_now = 1;
	del_timer(&osc->timer);
        kfree(osc);
        _MOD_DEC_USE(THIS_MODULE);
}

#ifdef CONFIG_SYSCTL
static int
ath_proc_read_nodes(struct ieee80211vap *vap, char *buf, int space)
{
        char *p = buf;
        struct ieee80211_node *ni;
        struct ath_node *an;
        struct maica_node *odst;
        struct ieee80211_node_table *nt =
                (struct ieee80211_node_table *) &vap->iv_ic->ic_sta;
        unsigned int x = 0;
	unsigned int this_tp, this_prob, this_eprob;
	struct ath_softc *sc = netdev_priv(vap->iv_ic->ic_dev);

        TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
                /* Assume each node needs 1500 bytes */
                if ((buf + space) < (p + 1500)) {
			if ((buf + space) > (p + 100)) {
				p += sprintf(p, "out of room for node %s\n\n", ether_sprintf(ni->ni_macaddr));
				break;
			}
			DPRINTF(sc, "%s: out of memeory to write tall of the nodes\n", __func__);
                        break;
		}
                an = ATH_NODE(ni);
                odst = ATH_NODE_MAICA(an);
                /* Skip ourself */
                if (memcmp(vap->iv_myaddr, ni->ni_macaddr, IEEE80211_ADDR_LEN) == 0)
                        continue;

                p += sprintf(p, "rate data for node:: %s\n", ether_sprintf(ni->ni_macaddr));
                p += sprintf(p, "rate     throughput  ewma prob   this prob  this succ/attempt   success    attempts\n");
                for (x = 0; x < odst->num_rates; x++) {
                        p += sprintf(p, "%s",
                                        (x == odst->current_rate) ? "T" : " ");
			
			p += sprintf(p, "%s",
				     (x == odst->max_tp_rate2) ? "t" : " ");

			p += sprintf(p, "%s",
				     (x == odst->max_prob_rate) ? "P" : " ");

                        p += sprintf(p, "%3d%s",
                                        odst->rates[x].rate/2,
                                        (odst->rates[x].rate & 0x1) != 0 ? ".5" : "  ");

			this_tp = ((odst->rs_this_tp[x] / 18000) * 96) >> 10;
			this_prob = odst->rs_thisprob[x] / 18;
			this_eprob = odst->rs_probability[x] / 18;
                        p += sprintf(p, "  %6u.%1u   %6u.%1u   %6u.%1u        %3u(%3u)   %8llu    %8llu\n",
				     this_tp / 10, this_tp % 10,
				     this_eprob / 10, this_eprob % 10,
				     this_prob / 10, this_prob % 10,
				     odst->rs_lastratesuccess[x],
				     odst->rs_lastrateattempts[x],
				     odst->rs_succ_hist[x],
				     odst->rs_att_hist[x]);
		}
		p += sprintf(p, "\n");

		p += sprintf(p, "Total packet count::    ideal %d  \n\n", odst->packet_count);
        }

        return (p - buf);
}

static int
ath_proc_ratesample_open(struct inode *inode, struct file *file)
{
        struct proc_ieee80211_priv *pv = NULL;
        struct proc_dir_entry *dp = PDE(inode);
        struct ieee80211vap *vap = dp->data;

        if (!(file->private_data = kmalloc(sizeof(struct proc_ieee80211_priv),
                                        GFP_KERNEL)))
                return -ENOMEM;

        /* initially allocate both read and write buffers */
        pv = (struct proc_ieee80211_priv *) file->private_data;
        memset(pv, 0, sizeof(struct proc_ieee80211_priv));
        pv->rbuf = vmalloc(MAX_PROC_IEEE80211_SIZE);
        if (!pv->rbuf) {
                kfree(pv);
                return -ENOMEM;
        }
        pv->wbuf = vmalloc(MAX_PROC_IEEE80211_SIZE);
        if (!pv->wbuf) {
                vfree(pv->rbuf);
                kfree(pv);
                return -ENOMEM;
        }

        memset(pv->wbuf, 0, MAX_PROC_IEEE80211_SIZE);
        memset(pv->rbuf, 0, MAX_PROC_IEEE80211_SIZE);
        pv->max_wlen = MAX_PROC_IEEE80211_SIZE;
        pv->max_rlen = MAX_PROC_IEEE80211_SIZE;

        /* now read the data into the buffer */
        pv->rlen = ath_proc_read_nodes(vap, pv->rbuf, MAX_PROC_IEEE80211_SIZE);
        return 0;
}

static struct file_operations ath_proc_ratesample_ops = {
        .read = NULL,
        .write = NULL,
        .open = ath_proc_ratesample_open,
        .release = NULL,
};

static void 
ath_rate_dynamic_proc_register(struct ieee80211vap *vap)
{
        /* Create proc entries for the rate control algorithm */
        ieee80211_proc_vcreate(vap, &ath_proc_ratesample_ops, "rate_info");
}

#endif /* CONFIG_SYSCTL */

static struct ieee80211_rate_ops ath_rate_ops = {
        .ratectl_id = IEEE80211_RATE_MAICA,
        .node_init = ath_rate_node_init,
        .node_cleanup = ath_rate_node_cleanup,
        .findrate = ath_rate_findrate,
        .setupxtxdesc = ath_rate_setupxtxdesc,
        .tx_complete = ath_rate_tx_complete,
        .newassoc = ath_rate_newassoc,
        .newstate = ath_rate_newstate,
        .attach = ath_rate_attach,
        .detach = ath_rate_detach,
        .dynamic_proc_register = ath_rate_dynamic_proc_register,
};

MODULE_AUTHOR("Sam Leffler/John Bicket/Derek Smithies/Duy Nguyen");
MODULE_DESCRIPTION("MAICA Rate bit-rate selection algorithm for Atheros devices");
#ifdef MODULE_VERSION
MODULE_VERSION(RELEASE_VERSION);
#endif
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static int __init ath_rate_maica_init(void)
{
	printk(KERN_INFO "%s: MAICA automatic rate control "
			"algorithm %s\n", dev_info, version);
	printk(KERN_INFO "%s: Max Segment size in the mrr set "
			"to %d us\n", dev_info, ath_segment_size);

        return ieee80211_rate_register(&ath_rate_ops);
}
module_init(ath_rate_maica_init);

static void __exit ath_rate_maica_exit(void)
{
        ieee80211_rate_unregister(&ath_rate_ops);
	printk(KERN_INFO "%s: unloaded\n", dev_info);
}
module_exit(ath_rate_maica_exit);

/* The comment below is magic for those who use emacs to edit this file. */
/* With the comment below, the tab key does auto indent to 8 spaces.     */
 
/*
 * Local Variables:
 * mode:c
 * c-file-style:linux
 * c-basic-offset:8
 * End:
 */
