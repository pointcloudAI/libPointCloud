#ifndef DLIB_AVX_MOD_Hh_
#define DLIB_AVX_MOD_Hh_

#include "simd_check.h"
#include "uintn.h"
#include <iostream>

#ifdef DLIB_HAVE_AVX
    //static inline uint32_t bit_scan_reverse (uint32_t a) __attribute__ ((pure));
    static inline uint32_t bit_scan_reverse (uint32_t a) {
		uint32_t retval;
#if defined(WINDOWS)
		__asm {
			bsr eax, a
			mov retval, eax
		}
#else
        __asm("bsrl %1, %0" : "=r"(retval) : "r"(a) : );
#endif
        return retval;
    }
    
    class simd_mod
    {
    private:
        __m256i m1;                                            // multiplier used in fast division
        __m256i d1;
        uint32_t shf1;
        uint32_t shf2;
        uint64_t tmp;

    public:
        void divisor(uint32_t d)
        {                                 // Set or change divisor, calculate parameters
            std::cout << "on divisor d : " << d << std::endl;
            uint32_t L, L2, sh1, sh2;
            uint64_t m;
            switch (d) {
                case 0:
                    m = sh1 = sh2 = 1 / d;                         // provoke error for d = 0
                    break;
                case 1:
                    m = 1; sh1 = sh2 = 0;                          // parameters for d = 1
                    break;
                case 2:
                    m = 1; sh1 = 1; sh2 = 0;                       // parameters for d = 2
                    break;
                default:                                           // general case for d > 2
                    std::cout << "before bit_scan_reverse 0" << std::endl;
                    L  = bit_scan_reverse(d-1)+1;                  // ceil(log2(d))
                    std::cout << "before bit_scan_reverse 1" << std::endl;
                    L2 = L < 32 ? 1 << L : 0;                      // 2^L, overflow to 0 if L = 32
                    std::cout << "before bit_scan_reverse 2" <<  std::endl;
                    std::cout << "L2  " << L2 << " d: " << d   <<  " L : " <<  L <<  std::endl;
                    std::cout << "uint32_t(L2 - d) << 4  " << (uint32_t(L2 - d) << 4)  <<   std::endl;
                    
                    tmp = uint64_t(L2 - d) << 32;
                    std::cout << "uint64_t(L2 - d) << 32  " << tmp  <<   std::endl;
                    std::cout << "uint64_t(L2 - d) << 32/d  " << tmp/d  <<   std::endl;
                    std::cout << "uint64_t(L2 - d) << 32/d  " << uint32_t(tmp/d)  <<   std::endl;
                    std::cout << "uint32_t((uint64_t(L2 - d) << 32) / d) " << uint32_t(tmp/d)  <<   std::endl;
                    std::cout << "1 + uint32_t((uint64_t(L2 - d) << 32) / d) " << 1 + uint32_t(tmp/d)  <<   std::endl;


                    m  = 1 + uint64_t((uint64_t(L2 - d) << 32) / d); // multiplier
                     std::cout << "before bit_scan_reverse 3" << std::endl;
                    sh1 = 1;
                     std::cout << "before bit_scan_reverse 4" << std::endl;
                    sh2 = L - 1;                         // shift counts
                     std::cout << "before bit_scan_reverse 5" << std::endl;
            }
            std::cout << "before bit_scan_reverse 6" << std::endl;
            //m1 = _mm256_set1_epi32(m);
            m1 = _mm256_set1_epi64x(m);
            // std::cout << "before bit_scan_reverse 7 m1 : " << m1  << std::endl;
            d1 = _mm256_set1_epi32(d);
            // std::cout << "before bit_scan_reverse 8 d1" << d1 << std::endl;
            shf1 = sh1;
            shf2 = sh2;
        }

        __m256i mod(__m256i lhs)
        {
            // set divisor
            //init(3);
            __m256i t1  = _mm256_mul_epu32(lhs,m1);                   // 32x32->64 bit unsigned multiplication of even elements of a
            __m256i t2  = _mm256_srli_epi64(t1,32);                // high dword of even numbered results
            __m256i t3  = _mm256_srli_epi64(lhs,32);                 // get odd elements of a into position for multiplication
            __m256i t4  = _mm256_mul_epu32(t3,m1);                  // 32x32->64 bit unsigned multiplication of odd elements
            __m256i t5  = _mm256_set_epi32(-1, 0, -1, 0, -1, 0, -1, 0);      // mask for odd elements
            __m256i t7  = _mm256_blendv_epi8(t2,t4,t5);            // blend two results
            __m256i t8  = _mm256_sub_epi32(lhs,t7);                  // subtract
            __m256i t9  = _mm256_srli_epi32(t8,shf1);          // shift right logical
            __m256i t10 = _mm256_add_epi32(t7,t9);                 // add
            __m256i t11 = _mm256_srli_epi32(t10,shf2);         // shift right logical
            __m256i t12 = _mm256_mullo_epi32(t11, d1);         // multiply quotient with divisor
            return _mm256_sub_epi32(lhs,t12);
            /*
            __m256i rem = _mm256_sub_epi32(lhs,t12);             // subtract
            
            int temp[8];
            _mm256_storeu_si256((__m256i*)temp,rem);
            std::cout << "(" << temp[0] << ", " << temp[1] << ", " << temp[2] << ", " << temp[3] << ", "
            << temp[4] << ", " << temp[5] << ", " << temp[6] << ", " << temp[7] << ")" << std::endl;
            */
        }
    };
#else

#endif
#endif // DLIB_simd16i_Hh_
