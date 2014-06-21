

#include "config.h"
#include "Clipboard.h"

#include "CachedImage.h"
#include "DOMImplementation.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "Image.h"

namespace WebCore {

Clipboard::Clipboard(ClipboardAccessPolicy policy, bool isForDragging) 
    : m_policy(policy)
    , m_dropEffect("none")
    , m_effectAllowed("uninitialized")
    , m_dragStarted(false)
    , m_forDragging(isForDragging)
    , m_dragImage(0)
{
}
    
void Clipboard::setAccessPolicy(ClipboardAccessPolicy policy)
{
    // once you go numb, can never go back
    ASSERT(m_policy != ClipboardNumb || policy == ClipboardNumb);
    m_policy = policy;
}

// These "conversion" methods are called by both WebCore and WebKit, and never make sense to JS, so we don't
// worry about security for these. They don't allow access to the pasteboard anyway.

static DragOperation dragOpFromIEOp(const String& op)
{
    // yep, it's really just this fixed set
    if (op == "uninitialized")
        return DragOperationEvery;
    if (op == "none")
        return DragOperationNone;
    if (op == "copy")
        return DragOperationCopy;
    if (op == "link")
        return DragOperationLink;
    if (op == "move")
        return DragOperationGeneric;    // FIXME: Why is this DragOperationGeneric? <http://webkit.org/b/33697>
    if (op == "copyLink")
        return (DragOperation)(DragOperationCopy | DragOperationLink);
    if (op == "copyMove")
        return (DragOperation)(DragOperationCopy | DragOperationGeneric | DragOperationMove);
    if (op == "linkMove")
        return (DragOperation)(DragOperationLink | DragOperationGeneric | DragOperationMove);
    if (op == "all")
        return DragOperationEvery;
    return DragOperationPrivate;  // really a marker for "no conversion"
}

static String IEOpFromDragOp(DragOperation op)
{
    bool moveSet = !!((DragOperationGeneric | DragOperationMove) & op);
    
    if ((moveSet && (op & DragOperationCopy) && (op & DragOperationLink))
        || (op == DragOperationEvery))
        return "all";
    if (moveSet && (op & DragOperationCopy))
        return "copyMove";
    if (moveSet && (op & DragOperationLink))
        return "linkMove";
    if ((op & DragOperationCopy) && (op & DragOperationLink))
        return "copyLink";
    if (moveSet)
        return "move";
    if (op & DragOperationCopy)
        return "copy";
    if (op & DragOperationLink)
        return "link";
    return "none";
}

DragOperation Clipboard::sourceOperation() const
{
    DragOperation op = dragOpFromIEOp(m_effectAllowed);
    ASSERT(op != DragOperationPrivate);
    return op;
}

DragOperation Clipboard::destinationOperation() const
{
    DragOperation op = dragOpFromIEOp(m_dropEffect);
    ASSERT(op == DragOperationCopy || op == DragOperationNone || op == DragOperationLink || op == DragOperationGeneric || op == DragOperationMove);
    return op;
}

void Clipboard::setSourceOperation(DragOperation op)
{
    ASSERT_ARG(op, op != DragOperationPrivate);
    m_effectAllowed = IEOpFromDragOp(op);
}

void Clipboard::setDestinationOperation(DragOperation op)
{
    ASSERT_ARG(op, op == DragOperationCopy || op == DragOperationNone || op == DragOperationLink || op == DragOperationGeneric || op == DragOperationMove);
    m_dropEffect = IEOpFromDragOp(op);
}

void Clipboard::setDropEffect(const String &effect)
{
    if (!m_forDragging)
        return;

    // The attribute must ignore any attempts to set it to a value other than none, copy, link, and move. 
    if (effect != "none" && effect != "copy"  && effect != "link" && effect != "move")
        return;

    if (m_policy == ClipboardReadable || m_policy == ClipboardTypesReadable)
        m_dropEffect = effect;
}

void Clipboard::setEffectAllowed(const String &effect)
{
    if (!m_forDragging)
        return;

    if (dragOpFromIEOp(effect) == DragOperationPrivate) {
        // This means that there was no conversion, and the effectAllowed that
        // we are passed isn't a valid effectAllowed, so we should ignore it,
        // and not set m_effectAllowed.

        // The attribute must ignore any attempts to set it to a value other than 
        // none, copy, copyLink, copyMove, link, linkMove, move, all, and uninitialized.
        return;
    }


    if (m_policy == ClipboardWritable)
        m_effectAllowed = effect;
}

} // namespace WebCore
