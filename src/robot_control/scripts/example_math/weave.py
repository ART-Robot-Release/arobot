# The example of weave

import scipy.weave as weave
import numpy as np
import time

def weave_sum(a):
    n=int(len(a))
    code="""
    int i;
    double counter;
    for(i = 0; i <= n; ++i){
        counter = counter + a(i);
    }
    return_val = counter; 
    """

    err = weave.inline(
        code, ['a', 'n'],
        type_converters = weave.converters.blitz,
        compiler="gcc"
        )
    return err

a = np.arange(0, 10000000, 1.0)

weave_sum(a)

start = time.clock()
for i in xrange(100):
    weave_sum(a)
print "weave sum:", (time.clock() - start) / 100.0

start = time.clock()
for i in xrange(100):
    np.sum(a)
print "np sum:", (time.clock() - start) / 100.0

start = time.clock()
print sum(a)
print "sum:", (time.clock() - start)

