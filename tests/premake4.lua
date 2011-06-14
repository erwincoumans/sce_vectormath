solution "tests"

	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "StaticRuntime", "NoRTTI", "NoExceptions"}
	configuration "Debug"
		flags { "Symbols", "StaticRuntime" , "NoRTTI", "NoExceptions"}
	platforms {"x32", "x64"}

	configuration "x64"		
		targetsuffix "_64"
	configuration {"x64", "debug"}
		targetsuffix "_x64_debug"
	configuration {"x64", "release"}
		targetsuffix "_x64"
	configuration {"x32", "debug"}
		targetsuffix "_debug"


	language "C++"
	location "build"
	targetdir "bin"

	project "test1_aos_scalar_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/scalar/cpp"	}
	files {		"test1_aos_cpp.cpp"	}

	project "test2_aos_scalar_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/scalar/cpp"	}
	files {		"test2_aos_cpp.cpp"	}

	project "test3_aos_scalar_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/scalar/cpp"	}
	files {		"test3_aos_cpp.cpp"	}

	project "test4_aos_scalar_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/scalar/cpp"	}
	files {		"test4_aos_cpp.cpp"	}

	project "test1_aos_SSE_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/SSE/cpp"	}
	files {		"test1_aos_cpp.cpp"	}
	
	project "test2_aos_SSE_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/SSE/cpp"	}
	files {		"test2_aos_cpp.cpp"	}
	
	project "test3_aos_SSE_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/SSE/cpp"	}
	files {		"test3_aos_cpp.cpp"	}
	
	project "test4_aos_SSE_cpp"
	kind "ConsoleApp"
	targetdir "bin"
	includedirs {"../include/vectormath/SSE/cpp"	}
	files {		"test4_aos_cpp.cpp"	}
	