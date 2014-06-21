

#ifndef StructureTransitionTable_h
#define StructureTransitionTable_h

#include "UString.h"
#include <wtf/HashFunctions.h>
#include <wtf/HashMap.h>
#include <wtf/HashTraits.h>
#include <wtf/PtrAndFlags.h>
#include <wtf/OwnPtr.h>
#include <wtf/RefPtr.h>

namespace JSC {

    class Structure;

    struct StructureTransitionTableHash {
        typedef std::pair<RefPtr<UString::Rep>, unsigned> Key;
        static unsigned hash(const Key& p)
        {
            return p.first->existingHash();
        }

        static bool equal(const Key& a, const Key& b)
        {
            return a == b;
        }

        static const bool safeToCompareToEmptyOrDeleted = true;
    };

    struct StructureTransitionTableHashTraits {
        typedef WTF::HashTraits<RefPtr<UString::Rep> > FirstTraits;
        typedef WTF::GenericHashTraits<unsigned> SecondTraits;
        typedef std::pair<FirstTraits::TraitType, SecondTraits::TraitType > TraitType;

        static const bool emptyValueIsZero = FirstTraits::emptyValueIsZero && SecondTraits::emptyValueIsZero;
        static TraitType emptyValue() { return std::make_pair(FirstTraits::emptyValue(), SecondTraits::emptyValue()); }

        static const bool needsDestruction = FirstTraits::needsDestruction || SecondTraits::needsDestruction;

        static void constructDeletedValue(TraitType& slot) { FirstTraits::constructDeletedValue(slot.first); }
        static bool isDeletedValue(const TraitType& value) { return FirstTraits::isDeletedValue(value.first); }
    };

    class StructureTransitionTable {
        typedef std::pair<Structure*, Structure*> Transition;
        typedef HashMap<StructureTransitionTableHash::Key, Transition, StructureTransitionTableHash, StructureTransitionTableHashTraits> TransitionTable;
    public:
        StructureTransitionTable() {
            m_transitions.m_singleTransition.set(0);
            m_transitions.m_singleTransition.setFlag(usingSingleSlot);
        }

        ~StructureTransitionTable() {
            if (!usingSingleTransitionSlot())
                delete table();
        }

        // The contains and get methods accept imprecise matches, so if an unspecialised transition exists
        // for the given key they will consider that transition to be a match.  If a specialised transition
        // exists and it matches the provided specificValue, get will return the specific transition.
        inline bool contains(const StructureTransitionTableHash::Key&, JSCell* specificValue);
        inline Structure* get(const StructureTransitionTableHash::Key&, JSCell* specificValue) const;
        inline bool hasTransition(const StructureTransitionTableHash::Key& key) const;
        void remove(const StructureTransitionTableHash::Key& key, JSCell* specificValue)
        {
            if (usingSingleTransitionSlot()) {
                ASSERT(contains(key, specificValue));
                setSingleTransition(0);
                return;
            }
            TransitionTable::iterator find = table()->find(key);
            if (!specificValue)
                find->second.first = 0;
            else
                find->second.second = 0;
            if (!find->second.first && !find->second.second)
                table()->remove(find);
        }
        void add(const StructureTransitionTableHash::Key& key, Structure* structure, JSCell* specificValue)
        {
            if (usingSingleTransitionSlot()) {
                if (!singleTransition()) {
                    setSingleTransition(structure);
                    return;
                }
                reifySingleTransition();
            }
            if (!specificValue) {
                TransitionTable::iterator find = table()->find(key);
                if (find == table()->end())
                    table()->add(key, Transition(structure, 0));
                else
                    find->second.first = structure;
            } else {
                // If we're adding a transition to a specific value, then there cannot be
                // an existing transition
                ASSERT(!table()->contains(key));
                table()->add(key, Transition(0, structure));
            }
        }

    private:
        TransitionTable* table() const { ASSERT(!usingSingleTransitionSlot()); return m_transitions.m_table; }
        Structure* singleTransition() const {
            ASSERT(usingSingleTransitionSlot());
            return m_transitions.m_singleTransition.get();
        }
        bool usingSingleTransitionSlot() const { return m_transitions.m_singleTransition.isFlagSet(usingSingleSlot); }
        void setSingleTransition(Structure* structure)
        { 
            ASSERT(usingSingleTransitionSlot());
            m_transitions.m_singleTransition.set(structure);
        }

        void setTransitionTable(TransitionTable* table)
        {
            ASSERT(usingSingleTransitionSlot());
#ifndef NDEBUG
            setSingleTransition(0);
#endif
            m_transitions.m_table = table;
            // This implicitly clears the flag that indicates we're using a single transition
            ASSERT(!usingSingleTransitionSlot());
        }
        inline void reifySingleTransition();

        enum UsingSingleSlot {
            usingSingleSlot
        };
        // Last bit indicates whether we are using the single transition optimisation
        union {
            TransitionTable* m_table;
            PtrAndFlagsBase<Structure, UsingSingleSlot> m_singleTransition;
        } m_transitions;
    };

} // namespace JSC

#endif // StructureTransitionTable_h