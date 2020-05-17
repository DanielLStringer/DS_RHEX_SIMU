# DS_RHEX_SIMU

Follow the instructions on https://github.com/AranBSmith/rhex-ite,
but instead of rhex_simu, install this code (DS_rhex_simu), 
and ignore the stuff called limbo and anything beneath this.



Move to etc./DS_rhex_simu/rhex_dart     

{ where:  export RESIBOTS_DIR=/etc.  }

Then compile this code with:

./waf configure --prefix=$RESIBOTS_DIR

./waf


To run the code:
For fixed version, just using tripod gait, no adaption, use:

./build/DS_fixed_test 0 1 raised.skel

For version that will adapt:

./build/DS_test 0 1 raised.skel

(change the first number, ie.0, to change the test environment, up to 12)

