
#ifndef UCNV_IO_H
#define UCNV_IO_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION

#include "udataswp.h"

#define UCNV_AMBIGUOUS_ALIAS_MAP_BIT 0x8000
#define UCNV_CONTAINS_OPTION_BIT 0x4000
#define UCNV_CONVERTER_INDEX_MASK 0xFFF
#define UCNV_NUM_RESERVED_TAGS 2
#define UCNV_NUM_HIDDEN_TAGS 1

enum {
    UCNV_IO_UNNORMALIZED,
    UCNV_IO_STD_NORMALIZED,
    UCNV_IO_NORM_TYPE_COUNT
};

typedef struct {
    uint16_t stringNormalizationType;
    uint16_t containsCnvOptionInfo;
} UConverterAliasOptions;

typedef struct UConverterAlias {
    const uint16_t *converterList;
    const uint16_t *tagList;
    const uint16_t *aliasList;
    const uint16_t *untaggedConvArray;
    const uint16_t *taggedAliasArray;
    const uint16_t *taggedAliasLists;
    const UConverterAliasOptions *optionTable;
    const uint16_t *stringTable;
    const uint16_t *normalizedStringTable;

    uint32_t converterListSize;
    uint32_t tagListSize;
    uint32_t aliasListSize;
    uint32_t untaggedConvArraySize;
    uint32_t taggedAliasArraySize;
    uint32_t taggedAliasListsSize;
    uint32_t optionTableSize;
    uint32_t stringTableSize;
    uint32_t normalizedStringTableSize;
} UConverterAlias;

#if U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define ucnv_io_stripForCompare ucnv_io_stripASCIIForCompare
#elif U_CHARSET_FAMILY==U_EBCDIC_FAMILY
#   define ucnv_io_stripForCompare ucnv_io_stripEBCDICForCompare
#else
#   error U_CHARSET_FAMILY is not valid
#endif

U_CAPI char * U_EXPORT2
ucnv_io_stripASCIIForCompare(char *dst, const char *name);

U_CAPI char * U_EXPORT2
ucnv_io_stripEBCDICForCompare(char *dst, const char *name);

U_CFUNC const char *
ucnv_io_getConverterName(const char *alias, UBool *containsOption, UErrorCode *pErrorCode);

U_CFUNC uint16_t
ucnv_io_countKnownConverters(UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
ucnv_swapAliases(const UDataSwapper *ds,
                 const void *inData, int32_t length, void *outData,
                 UErrorCode *pErrorCode);

#endif

#endif /* _UCNV_IO */

