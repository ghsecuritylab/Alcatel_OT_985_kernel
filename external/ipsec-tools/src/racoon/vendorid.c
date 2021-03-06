/*	$NetBSD: vendorid.c,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: vendorid.c,v 1.10 2006/02/22 16:10:21 vanhu Exp */


#include "config.h"

#include <sys/types.h>
#include <sys/param.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "var.h"
#include "misc.h"
#include "vmbuf.h"
#include "plog.h"
#include "debug.h"

#include "localconf.h"
#include "isakmp_var.h"
#include "isakmp.h"
#include "vendorid.h"
#include "crypto_openssl.h"

static struct vendor_id all_vendor_ids[] = {
{ VENDORID_IPSEC_TOOLS, "IPSec-Tools" },
{ VENDORID_GSSAPI_LONG, "A GSS-API Authentication Method for IKE" },
{ VENDORID_GSSAPI     , "GSSAPI" },
{ VENDORID_MS_NT5     , "MS NT5 ISAKMPOAKLEY" },
{ VENDORID_NATT_00    , "draft-ietf-ipsec-nat-t-ike-00" },
{ VENDORID_NATT_01    , "draft-ietf-ipsec-nat-t-ike-01" },
{ VENDORID_NATT_02    , "draft-ietf-ipsec-nat-t-ike-02" },
{ VENDORID_NATT_02_N  , "draft-ietf-ipsec-nat-t-ike-02\n" },
{ VENDORID_NATT_03    , "draft-ietf-ipsec-nat-t-ike-03" },
{ VENDORID_NATT_04    , "draft-ietf-ipsec-nat-t-ike-04" },
{ VENDORID_NATT_05    , "draft-ietf-ipsec-nat-t-ike-05" },
{ VENDORID_NATT_06    , "draft-ietf-ipsec-nat-t-ike-06" },
{ VENDORID_NATT_07    , "draft-ietf-ipsec-nat-t-ike-07" },
{ VENDORID_NATT_08    , "draft-ietf-ipsec-nat-t-ike-08" },
{ VENDORID_NATT_RFC   , "RFC 3947" },
{ VENDORID_XAUTH      , "draft-ietf-ipsra-isakmp-xauth-06.txt" },
{ VENDORID_UNITY      , "CISCO-UNITY" },
{ VENDORID_FRAG       , "FRAGMENTATION" },
/* Just a readable string for DPD ... */
{ VENDORID_DPD        , "DPD" },
/* Other known Vendor IDs */
{ VENDORID_KAME       , "KAME/racoon" },
};

#define NUMVENDORIDS	(sizeof(all_vendor_ids)/sizeof(all_vendor_ids[0]))

#define DPD_MAJOR_VERSION	0x01
#define DPD_MINOR_VERSION	0x00

const char vendorid_dpd_hash[] = {
	0xAF, 0xCA, 0xD7, 0x13,
	0x68, 0xA1, 0xF1, 0xC9,
	0x6B, 0x86, 0x96, 0xFC,
	0x77, 0x57, DPD_MAJOR_VERSION, DPD_MINOR_VERSION
};


static vchar_t *vendorid_fixup(int, vchar_t *t);

static struct vendor_id *
lookup_vendor_id_by_id (int id)
{
	int i;

	for (i = 0; i < NUMVENDORIDS; i++)
		if (all_vendor_ids[i].id == id)
			return &all_vendor_ids[i];

	return NULL;
}

const char *
vid_string_by_id (int id)
{
	struct vendor_id *current;

	if (id == VENDORID_DPD)
		return vendorid_dpd_hash;

	current = lookup_vendor_id_by_id(id);

	return current ? current->string : NULL;
}

static struct vendor_id *
lookup_vendor_id_by_hash (const char *hash)
{
	int i;
	unsigned char *h = (unsigned char *)hash;

	for (i = 0; i < NUMVENDORIDS; i++)
		if (strncmp(all_vendor_ids[i].hash->v, hash,
			    all_vendor_ids[i].hash->l) == 0)
			return &all_vendor_ids[i];

	return NULL;
}

void
compute_vendorids (void)
{
	int i;
	vchar_t vid;

	for (i = 0; i < NUMVENDORIDS; i++) {
		/* VENDORID_DPD is not a MD5 sum... */
		if(all_vendor_ids[i].id == VENDORID_DPD){
			all_vendor_ids[i].hash = vmalloc(sizeof(vendorid_dpd_hash));
			if (all_vendor_ids[i].hash == NULL) {
				plog(LLV_ERROR, LOCATION, NULL,
					"unable to get memory for VID hash\n");
				exit(1); /* this really shouldn't happen */
			}
			memcpy(all_vendor_ids[i].hash->v, vendorid_dpd_hash,
				   sizeof(vendorid_dpd_hash));
			continue;
		}

		vid.v = (char *) all_vendor_ids[i].string;
		vid.l = strlen(vid.v);

		all_vendor_ids[i].hash = eay_md5_one(&vid);
		if (all_vendor_ids[i].hash == NULL)
			plog(LLV_ERROR, LOCATION, NULL,
			    "unable to hash vendor ID string\n");

		/* Special cases */
		all_vendor_ids[i].hash =
			vendorid_fixup(all_vendor_ids[i].id,
				       all_vendor_ids[i].hash);
	}
}

vchar_t *
set_vendorid(int vendorid)
{
	struct vendor_id *current;
	vchar_t vid, *new;

	if (vendorid == VENDORID_UNKNOWN) {
		/*
		 * The default unknown ID gets translated to
		 * KAME/racoon.
		 */
		vendorid = VENDORID_DEFAULT;
	}

	current = lookup_vendor_id_by_id(vendorid);
	if (current == NULL) {
		plog(LLV_ERROR, LOCATION, NULL,
		    "invalid vendor ID index: %d\n", vendorid);
		return (NULL);
	}

	/* The rest of racoon expects a private copy 
	 * of the VID that could be free'd after use.
	 * That's why we don't return the original pointer. */
	return vdup(current->hash);
}

int
check_vendorid(struct isakmp_gen *gen)
{
	vchar_t vid, *vidhash;
	int i, vidlen;
	struct vendor_id *current;

	if (gen == NULL)
		return (VENDORID_UNKNOWN);

	vidlen = ntohs(gen->len) - sizeof(*gen);

	current = lookup_vendor_id_by_hash((char *)(gen + 1));
	if (!current)
		goto unknown;
	
	if (current->hash->l < vidlen)
		plog(LLV_INFO, LOCATION, NULL,
		     "received broken Microsoft ID: %s\n",
		     current->string);
	else
		plog(LLV_INFO, LOCATION, NULL,
		     "received Vendor ID: %s\n",
		     current->string);

	return current->id;

unknown:
	plog(LLV_DEBUG, LOCATION, NULL, "received unknown Vendor ID\n");
	plogdump(LLV_DEBUG, (char *)(gen + 1), vidlen);
	return (VENDORID_UNKNOWN);
}

static vchar_t * 
vendorid_fixup(vendorid, vidhash)
	int vendorid;		 
	vchar_t *vidhash;
{			   
	switch(vendorid) {
	case VENDORID_XAUTH: {	/* The vendor Id is truncated */
		vchar_t *tmp;					    
				  
		if ((tmp = vmalloc(8)) == NULL) {
			plog(LLV_ERROR, LOCATION, NULL,
			    "unable to hash vendor ID string\n");
			return NULL;				    
		}			
		  
		memcpy(tmp->v, vidhash->v, 8);
		vfree(vidhash);		  
		vidhash = tmp;
				   
		break;
	} 
	case VENDORID_UNITY:	/* Two bytes tweak */
		vidhash->v[14] = 0x01;		  
		vidhash->v[15] = 0x00;
		break;		   

	default:     
		break;
	}		
	
	return vidhash;
}			 
