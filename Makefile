all: feat_match

feat_match: feat_match.cpp
	g++ -std=c++11 `pkg-config --libs opencv` feat_match.cpp -o feat_match -O3
