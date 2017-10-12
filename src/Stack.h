
#pragma once


template<typename T>
struct StackItemT {

  inline static void xchg(StackItemT& a, StackItemT&b) {

    assert(sizeof(T) <= 12);
    
    const vfloat4 sse_a = vfloat4::load((void*)&a);
    const vfloat4 sse_b = vfloat4::load((void*)&b);
    vfloat4::store((void*)&a,sse_b);
    vfloat4::store((void*)&b,sse_a);
  }

  /// this is slow, should be repleaced eventually ///
  inline static void xchg(StackItemT* a, StackItemT* b) {
    std::iter_swap(a,b);
  }

  inline friend void sort(StackItemT& s1, StackItemT &s2) {
    if (s2.dist < s1.dist) xchg(s2,s1);
  }

  inline friend void sort(StackItemT& s1, StackItemT &s2, StackItemT &s3) {
    if (s2.dist <= s1.dist) xchg(&s2,&s1);
    if (s3.dist <= s2.dist) xchg(&s3,&s2);
    if (s2.dist <= s1.dist) xchg(&s2,&s1);	
  }

  inline friend void sort(StackItemT& s1, StackItemT& s2, StackItemT& s3, StackItemT& s4) {
    if (s2.dist < s1.dist) xchg(&s2,&s1);
    if (s4.dist < s3.dist) xchg(&s4,&s3);
    if (s3.dist < s1.dist) xchg(&s3,&s1);
    if (s4.dist < s2.dist) xchg(&s4,&s2);
    if (s3.dist < s2.dist) xchg(&s3,&s2);
  }

  inline friend void sort(StackItemT* begin, StackItemT* end) {
    for (StackItemT* i = begin+1; i != end; ++i)
      {
        const vfloat4 item = vfloat4::load((void*)i);
        const unsigned dist = i->dist;
        StackItemT* j = i;

        while ((j != begin) && ((j-1)->dist < dist))
	  {
	    vfloat4::store(j, vfloat4::load((void*)(j-1)));
	    --j;
	  }

        vfloat4::store(j, item);
      }
  }

public:
  T ptr;
  unsigned dist;

};
