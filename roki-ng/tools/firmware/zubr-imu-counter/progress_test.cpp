#include "ImuProgress.h"
#include <cassert>
int main() {
 ImuProgress p{};
 assert(p.sequence == 0);
 p.observe(100); assert(p.sequence == 1);
 p.observe(100); assert(p.sequence == 1);
 p.observe(101); assert(p.sequence == 2);
 p.sequence = 0xffffffffu;
 p.observe(102); assert(p.sequence == 0); // unsigned wrap
 p.observe(1); assert(p.sequence == 1); // restarted sensor timestamp
}
