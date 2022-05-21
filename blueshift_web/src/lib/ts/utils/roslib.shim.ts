// By default, the front end version of ROSLIB creates a global variable called window.ROSLIB that contains all of ROSLIB
// This shim allows us to import ROSLIB with a more normal syntax without having to touch the global variable
// The global variable still exists, but you shouldn't use it. Instead, you should import ROSLIB as follows:
// import ROSLIB from "$lib/ts/utils/roslib.shim";

// This should work even if the global variable is already defined (ie if you imported it in multiple places), but it hasn't been tested thoroughly

import "roslib/build/roslib"

const ROSLIB = window.ROSLIB;

export default ROSLIB;