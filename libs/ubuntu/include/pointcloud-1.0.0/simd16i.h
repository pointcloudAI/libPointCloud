// Copyright (C) 2013  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef DLIB_simd16i_Hh_
#define DLIB_simd16i_Hh_

#include "simd_check.h"
#include "uintn.h"

namespace dlib
{

#ifdef DLIB_HAVE_AVX
    class simd16i
    {
    public:
        typedef int16 type;

        inline simd16i() {}
        inline simd16i(int16 f) { x = _mm256_set1_epi16(f); }
        inline simd16i(int16 r0, int16 r1, int16 r2, int16 r3,
               int16 r4, int16 r5, int16 r6, int16 r7,int16 r8, int16 r9, int16 r10, int16 r11,
               int16 r12, int16 r13, int16 r14, int16 r15 ) 
        { x = _mm256_setr_epi16(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15); }

        inline simd16i(const __m256i& val):x(val) {}
        /*
        inline simd16i(const simd4i& low, const simd4i& high)
        {
            x = _mm256_insertf128_si256(_mm256_castsi128_si256(low),high,1);
        }
        */
        inline simd16i& operator=(const __m256i& val)
        {
            x = val;
            return *this;
        }

        inline operator __m256i() const { return x; }

        inline void load_aligned(const type* ptr)  { x = _mm256_load_si256((const __m256i*)ptr); }
        inline void store_aligned(type* ptr) const { _mm256_store_si256((__m256i*)ptr, x); }
        inline void load(const type* ptr)          { x = _mm256_loadu_si256((const __m256i*)ptr); }
        inline void store(type* ptr)         const { _mm256_storeu_si256((__m256i*)ptr, x); }
        /*
        inline simd4i low() const { return _mm256_castsi256_si128(x); }
        inline simd4i high() const { return _mm256_extractf128_si256(x,1); }
        */
        inline unsigned int size() const { return 16; }
        inline int16 operator[](unsigned int idx) const 
        {
            int16 temp[16];
            store(temp);
            return temp[idx];
        }

    private:
        __m256i x;
    };
#else
    /*
    class simd16i
    {
    public:
        typedef int16 type;

        inline simd16i() {}
        
        //inline simd16i(const simd4i& low_, const simd4i& high_): _low(low_),_high(high_){}
        
        inline simd16i(int16 f) :_low(f),_high(f) {}
        inline simd16i(int16 r0, int16 r1, int16 r2, int16 r3, int16 r4, int16 r5, int16 r6, int16 r7) :
            _low(r0,r1,r2,r3), _high(r4,r5,r6,r7) {}

        struct rawarray
        {
            simd4i low, high;
        };
        inline simd16i(const rawarray& a) 
        { 
            _low = a.low;
            _high = a.high;
        }

        inline void load_aligned(const type* ptr)  { _low.load_aligned(ptr); _high.load_aligned(ptr+4); }
        inline void store_aligned(type* ptr) const { _low.store_aligned(ptr); _high.store_aligned(ptr+4); }
        inline void load(const type* ptr)          { _low.load(ptr); _high.load(ptr+4); }
        inline void store(type* ptr)         const { _low.store(ptr); _high.store(ptr+4); }

        inline unsigned int size() const { return 8; }
        inline int16 operator[](unsigned int idx) const 
        {
            if (idx < 4)
                return _low[idx];
            else
                return _high[idx-4];
        }

        inline const simd4i& low() const { return _low; }
        inline const simd4i& high() const { return _high; }

    private:
        simd4i _low, _high;
    };*/

#endif

// ----------------------------------------------------------------------------------------

    inline std::ostream& operator<<(std::ostream& out, const simd16i& item)
    {
        int16 temp[16];
        item.store(temp);
        out << "(" << temp[0] << ", " << temp[1] << ", " << temp[2] << ", " << temp[3] << ", "
                   << temp[4] << ", " << temp[5] << ", " << temp[6] << ", " << temp[7] << ","
                   << temp[8] << ", " << temp[9] << ", " << temp[10] << ", " << temp[11] << ","
                   << temp[12] << ", " << temp[13] << ", " << temp[14] << ", " << temp[15] << ")";
        return out;
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator+ (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_add_epi16(lhs, rhs); 
#else
        return simd16i(lhs.low()+rhs.low(),
                      lhs.high()+rhs.high());
#endif
    }

    inline simd16i& operator+= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs + rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator- (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_sub_epi16(lhs, rhs); 
#else
        return simd16i(lhs.low()-rhs.low(),
                      lhs.high()-rhs.high());
#endif
    }
    
    inline simd16i& operator-= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs - rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator* (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_mullo_epi16(lhs, rhs); 
#else
        return simd16i(lhs.low()*rhs.low(),
                      lhs.high()*rhs.high());
#endif
    }
    inline simd16i& operator*= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs * rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator& (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_and_si256(lhs, rhs); 
#else
        return simd16i(lhs.low()&rhs.low(),
                      lhs.high()&rhs.high());
#endif
    }

    inline simd16i& operator&= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs & rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator| (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_or_si256(lhs, rhs); 
#else
        return simd16i(lhs.low()|rhs.low(),
                      lhs.high()|rhs.high());
#endif
    }
    inline simd16i& operator|= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs | rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator^ (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_xor_si256(lhs, rhs); 
#else
        return simd16i(lhs.low()^rhs.low(),
                      lhs.high()^rhs.high());
#endif
    }
    inline simd16i& operator^= (simd16i& lhs, const simd16i& rhs) 
    { return lhs = lhs ^ rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator~ (const simd16i& lhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_xor_si256(lhs, _mm256_set1_epi16(0xFFFFFFFF)); 
#else
        return simd16i(~lhs.low(), ~lhs.high());
#endif
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator<< (const simd16i& lhs, const int& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_sll_epi16(lhs,_mm_cvtsi32_si128(rhs));
#else
        return simd16i(lhs.low()<<rhs,
                      lhs.high()<<rhs);
#endif

    }
    inline simd16i& operator<<= (simd16i& lhs, const int& rhs) 
    { return lhs = lhs << rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator>> (const simd16i& lhs, const int& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_sra_epi16(lhs,_mm_cvtsi32_si128(rhs));
#else
        return simd16i(lhs.low()>>rhs,
                      lhs.high()>>rhs);
#endif
    }
    inline simd16i& operator>>= (simd16i& lhs, const int& rhs) 
    { return lhs = lhs >> rhs; return lhs;}

// ----------------------------------------------------------------------------------------

    inline simd16i operator== (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_cmpeq_epi16(lhs, rhs); 
#else
        return simd16i(lhs.low()==rhs.low(),
                      lhs.high()==rhs.high());
#endif
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator!= (const simd16i& lhs, const simd16i& rhs) 
    { 
        return ~(lhs==rhs);
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator> (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_cmpgt_epi16(lhs, rhs); 
#else
        return simd16i(lhs.low()>rhs.low(),
                      lhs.high()>rhs.high());
#endif
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator< (const simd16i& lhs, const simd16i& rhs) 
    { 
        return rhs > lhs;
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator<= (const simd16i& lhs, const simd16i& rhs) 
    { 
        return ~(lhs > rhs); 
    }

// ----------------------------------------------------------------------------------------

    inline simd16i operator>= (const simd16i& lhs, const simd16i& rhs) 
    { 
        return rhs <= lhs;
    }

// ----------------------------------------------------------------------------------------

    inline simd16i min (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_min_epi16(lhs, rhs); 
#else
        return simd16i(min(lhs.low(),rhs.low()),
                      min(lhs.high(),rhs.high()));
#endif
    }

// ----------------------------------------------------------------------------------------

    inline simd16i max (const simd16i& lhs, const simd16i& rhs) 
    { 
#ifdef DLIB_HAVE_AVX2
        return _mm256_max_epi16(lhs, rhs); 
#else
        return simd16i(max(lhs.low(),rhs.low()),
                      max(lhs.high(),rhs.high()));
#endif
    }

// ----------------------------------------------------------------------------------------

    inline int16 sum(const simd16i& item)
    {
        int16 temp[16];
        item.store(temp);
        return temp[0] + temp[1] + temp[2] + temp[3] +
               temp[4] + temp[5] + temp[6] + temp[7] +
               temp[8] + temp[9] + temp[10] + temp[11] +
               temp[12] + temp[13] + temp[14] + temp[15];
        //return sum(item.low()+item.high());
    }

// ----------------------------------------------------------------------------------------

    // perform cmp ? a : b
    inline simd16i select(const simd16i& cmp, const simd16i& a, const simd16i& b)
    {
#ifdef DLIB_HAVE_AVX2
        return _mm256_blendv_epi8(b,a,cmp);
#else
        return simd16i(select(cmp.low(),  a.low(),  b.low()),
                      select(cmp.high(), a.high(), b.high()));
#endif
    }

// ----------------------------------------------------------------------------------------

}

#endif // DLIB_simd16i_Hh_


