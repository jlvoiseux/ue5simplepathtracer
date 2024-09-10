# A Simple Path Tracer for Unreal Engine 5

First pass - set the color arrays to the output of the gbuffers
	We need to have a Gbuffers material that outputs to a separate render texture (created on begin play)
	This render texture is then used to init the pathtracing arrays
	We keep the current setup where the output render texture outputs to a post-process material
	Path tracing then starts