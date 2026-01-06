Overview: Using a tool called callgrind that belongs in valgrind’s suite of tools to analyze instruction count % for functions, and visualize significant bottle neck’s control flow.

Prerequisites: Install valgrind via apt-get on the ubuntu docker container that is used to run the vamp builds.
Install kcachegrind or qcachegrind onto any machine to open up the callgrind files.  kcachegrind is the version with more features, but because I’m using a Mac, I just use qcachegrind. 

Note: My personal workflow is to run callgrind in the docker container to output an analysis file. I then open the analysis file outside of my docker container using qcachegrind

Instructions:
1. The C++ code must be compiled with the -g flag, which is the debug flag in order to see function names. Otherwise, callgrind won’t output human readable analysis files. 
Run:
cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug .
cmake --build build


2. Run something like ./build/vamp_rrtc_example will callgrind
Command to run: valgrind —tool=callgrind [Path to executable]

3. The output file will be created, probably named “callgrind.out.###” Probably rename the file to something more memorable.

4. Launch the Application. qcachegrind/kcachegrind via a terminal command.
Command to run: qcachegrind

5. From the GUI, open a file and select on the callgrind file you want to analyze.

Note: Once there, I find the call graph to be the most useful tool. 