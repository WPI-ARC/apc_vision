# apc_vision
CV code for the Amazon Picking Challenge

This branch contains the code used to test and produce the confusion matrix for
the feature matching method. When running the program, pass it a path to a directory
containing folders with the berkeley images for each object (which objects are used
can be modified via the config.yaml file). This program also expects to find calibN.jpeg
files in each of the object directories; these are the reference images used. N should be
an integer starting at 0. The number of reference images expected for each object is 
configured via the config.yaml file.
