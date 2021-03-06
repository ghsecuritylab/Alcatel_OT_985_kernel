

#ifndef WTF_PassRefPtr_h
#define WTF_PassRefPtr_h

#include "AlwaysInline.h"

namespace WTF {

    template<typename T> class RefPtr;
    template<typename T> class PassRefPtr;
    template <typename T> PassRefPtr<T> adoptRef(T*);


    // Remove inline for WINSCW compiler to prevent the compiler agressively resolving
    // T::ref() and T::deref(), which will fail compiling when PassRefPtr<T> is used as
    // a class member or function arguments before T is defined.
    template<typename T>
#if !COMPILER(WINSCW)
    inline
#endif
    void refIfNotNull(T* ptr)
    {
        if (UNLIKELY(ptr != 0))
            ptr->ref();
    }

    template<typename T> 
#if !COMPILER(WINSCW)
    inline 
#endif
    void derefIfNotNull(T* ptr)
    {
        if (UNLIKELY(ptr != 0))
            ptr->deref();
    }

    template<typename T> class PassRefPtr {
    public:
        PassRefPtr() : m_ptr(0) {}
        PassRefPtr(T* ptr) : m_ptr(ptr) { refIfNotNull(ptr); }
        // It somewhat breaks the type system to allow transfer of ownership out of
        // a const PassRefPtr. However, it makes it much easier to work with PassRefPtr
        // temporaries, and we don't really have a need to use real const PassRefPtrs 
        // anyway.
        PassRefPtr(const PassRefPtr& o) : m_ptr(o.releaseRef()) {}
        template <typename U> PassRefPtr(const PassRefPtr<U>& o) : m_ptr(o.releaseRef()) { }

        ALWAYS_INLINE ~PassRefPtr() { derefIfNotNull(m_ptr); }

        template <class U> 
        PassRefPtr(const RefPtr<U>& o) : m_ptr(o.get()) { T* ptr = m_ptr; refIfNotNull(ptr); }
        
        T* get() const { return m_ptr; }

        void clear() { T* ptr = m_ptr; derefIfNotNull(ptr); m_ptr = 0; }
        T* releaseRef() const { T* tmp = m_ptr; m_ptr = 0; return tmp; }

        T& operator*() const { return *m_ptr; }
        T* operator->() const { return m_ptr; }
        
        bool operator!() const { return !m_ptr; }

        // This conversion operator allows implicit conversion to bool but not to other integer types.
        typedef T* (PassRefPtr::*UnspecifiedBoolType);
        operator UnspecifiedBoolType() const { return m_ptr ? &PassRefPtr::m_ptr : 0; }

        PassRefPtr& operator=(T*);
        PassRefPtr& operator=(const PassRefPtr&);
        template <typename U> PassRefPtr& operator=(const PassRefPtr<U>&);
        template <typename U> PassRefPtr& operator=(const RefPtr<U>&);

        friend PassRefPtr adoptRef<T>(T*);
    private:
        // adopting constructor
        PassRefPtr(T* ptr, bool) : m_ptr(ptr) {}
        mutable T* m_ptr;
    };
    
    // NonNullPassRefPtr: Optimized for passing non-null pointers. A NonNullPassRefPtr
    // begins life non-null, and can only become null through a call to releaseRef()
    // or clear().

    // FIXME: NonNullPassRefPtr could just inherit from PassRefPtr. However,
    // if we use inheritance, GCC's optimizer fails to realize that destruction
    // of a released NonNullPassRefPtr is a no-op. So, for now, just copy the
    // most important code from PassRefPtr.
    template <typename T> class NonNullPassRefPtr {
    public:
        NonNullPassRefPtr(T* ptr)
            : m_ptr(ptr)
        {
            ASSERT(m_ptr);
            m_ptr->ref();
        }

        template <class U> NonNullPassRefPtr(const RefPtr<U>& o)
            : m_ptr(o.get())
        {
            ASSERT(m_ptr);
            m_ptr->ref();
        }

        NonNullPassRefPtr(const NonNullPassRefPtr& o)
            : m_ptr(o.releaseRef())
        {
            ASSERT(m_ptr);
        }

        template <class U> NonNullPassRefPtr(const NonNullPassRefPtr<U>& o)
            : m_ptr(o.releaseRef())
        {
            ASSERT(m_ptr);
        }

        template <class U> NonNullPassRefPtr(const PassRefPtr<U>& o)
            : m_ptr(o.releaseRef())
        {
            ASSERT(m_ptr);
        }

        ALWAYS_INLINE ~NonNullPassRefPtr() { derefIfNotNull(m_ptr); }

        T* get() const { return m_ptr; }

        void clear() { derefIfNotNull(m_ptr); m_ptr = 0; }
        T* releaseRef() const { T* tmp = m_ptr; m_ptr = 0; return tmp; }

        T& operator*() const { return *m_ptr; }
        T* operator->() const { return m_ptr; }

    private:
        mutable T* m_ptr;
    };

    template <typename T> template <typename U> inline PassRefPtr<T>& PassRefPtr<T>::operator=(const RefPtr<U>& o) 
    {
        T* optr = o.get();
        refIfNotNull(optr);
        T* ptr = m_ptr;
        m_ptr = optr;
        derefIfNotNull(ptr);
        return *this;
    }
    
    template <typename T> inline PassRefPtr<T>& PassRefPtr<T>::operator=(T* optr)
    {
        refIfNotNull(optr);
        T* ptr = m_ptr;
        m_ptr = optr;
        derefIfNotNull(ptr);
        return *this;
    }

    template <typename T> inline PassRefPtr<T>& PassRefPtr<T>::operator=(const PassRefPtr<T>& ref)
    {
        T* ptr = m_ptr;
        m_ptr = ref.releaseRef();
        derefIfNotNull(ptr);
        return *this;
    }
    
    template <typename T> template <typename U> inline PassRefPtr<T>& PassRefPtr<T>::operator=(const PassRefPtr<U>& ref)
    {
        T* ptr = m_ptr;
        m_ptr = ref.releaseRef();
        derefIfNotNull(ptr);
        return *this;
    }
    
    template <typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, const PassRefPtr<U>& b) 
    { 
        return a.get() == b.get(); 
    }

    template <typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, const RefPtr<U>& b) 
    { 
        return a.get() == b.get(); 
    }

    template <typename T, typename U> inline bool operator==(const RefPtr<T>& a, const PassRefPtr<U>& b) 
    { 
        return a.get() == b.get(); 
    }

    template <typename T, typename U> inline bool operator==(const PassRefPtr<T>& a, U* b) 
    { 
        return a.get() == b; 
    }
    
    template <typename T, typename U> inline bool operator==(T* a, const PassRefPtr<U>& b) 
    {
        return a == b.get(); 
    }
    
    template <typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, const PassRefPtr<U>& b) 
    { 
        return a.get() != b.get(); 
    }

    template <typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, const RefPtr<U>& b) 
    { 
        return a.get() != b.get(); 
    }

    template <typename T, typename U> inline bool operator!=(const RefPtr<T>& a, const PassRefPtr<U>& b) 
    { 
        return a.get() != b.get(); 
    }

    template <typename T, typename U> inline bool operator!=(const PassRefPtr<T>& a, U* b)
    {
        return a.get() != b; 
    }

    template <typename T, typename U> inline bool operator!=(T* a, const PassRefPtr<U>& b) 
    { 
        return a != b.get(); 
    }
    
    template <typename T> inline PassRefPtr<T> adoptRef(T* p)
    {
        return PassRefPtr<T>(p, true);
    }

    template <typename T, typename U> inline PassRefPtr<T> static_pointer_cast(const PassRefPtr<U>& p) 
    { 
        return adoptRef(static_cast<T*>(p.releaseRef())); 
    }

    template <typename T, typename U> inline PassRefPtr<T> const_pointer_cast(const PassRefPtr<U>& p) 
    { 
        return adoptRef(const_cast<T*>(p.releaseRef())); 
    }

    template <typename T> inline T* getPtr(const PassRefPtr<T>& p)
    {
        return p.get();
    }

} // namespace WTF

using WTF::PassRefPtr;
using WTF::NonNullPassRefPtr;
using WTF::adoptRef;
using WTF::static_pointer_cast;
using WTF::const_pointer_cast;

#endif // WTF_PassRefPtr_h
