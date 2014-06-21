

#ifndef _ENCOLL
#define _ENCOLL

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "tscoll.h"

class CollationEnglishTest: public IntlTestCollator {
public:
    // If this is too small for the test data, just increase it.
    // Just don't make it too large, otherwise the executable will get too big
    enum EToken_Len { MAX_TOKEN_LEN = 16 };

    CollationEnglishTest();
    virtual ~CollationEnglishTest();
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );

    // performs test with strength PRIMARY
    void TestPrimary(/* char* par */);

    // perform test with strength SECONDARY
    void TestSecondary(/* char* par */);

    // perform test with strength TERTIARY
    void TestTertiary(/* char* par */);

private:
    Collator *myCollation;
};

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
