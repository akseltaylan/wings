#include <math.h>
#include "spring.h"
#include "io.h"

#define SIM true // TODO: remove (debugging)

// sim variables
float mass = 2.5f;
int n = 0;
float t = 0.0f;
float ks = 0.5f;
float kd = 0.5f;
float amp = 30.0f;
float restLength = 0.1f;
int startFrame = 1;
int frameMax = 240;
const static float h = 0.005f;
const static float fps = 24.0f;
const static float frameCheck = 1.0f/fps;
const static int displayCheck = frameCheck / h;
const Eigen::Vector3f gravity(0.0f,-0.098f,0.0f);

// state variables
int numPoints = 2;
int amtValues = 10;
int stateLen;
// state contains current position data, velocity data, and force data
float * state = new float[stateLen];
std::vector<Spring> springs;

// this vector holds data read in from houdini
std::vector<float *> frameStates;

void addFrameState(std::vector<HoudiniVertData> & framedata) {
	float * frameState = new float[stateLen];
	int i = 0;
	for (HoudiniVertData hdata : framedata) {
		// position
		frameState[i] = hdata.pos.x(); 
		frameState[i+1] = hdata.pos.y();
		frameState[i+2] = hdata.pos.z();
		// initialize velocity
		frameState[i+3] = 0.0; 
		frameState[i+4] = 0.0;
		frameState[i+5] = 0.0;
		// initialize force
		frameState[i+6] = 0.0; 
		frameState[i+7] = 0.0;
		frameState[i+8] = 0.0;
		// is root?
		frameState[i+9] = hdata.pt_type == 'r' ? 1.0f : 0.0f;
		i += amtValues;
	}
	frameStates.push_back(frameState);
}

void init(IO * io) {
	const std::string outfolder = "in_data/";
	std::string filepath;
	for (int i = startFrame; i <= frameMax; ++i) {
		filepath = outfolder + "frame" + std::to_string(i) + ".txt"; 
		std::vector<HoudiniVertData> framedata = io->read_data(filepath.c_str());
		if (numPoints != framedata.size()) {
			numPoints = framedata.size();
			stateLen = numPoints * amtValues;
		}
		// add this frame's state to the vector of states
		addFrameState(framedata);
	}
	state = frameStates[0];
}

void initSprings() {
	// edge springs
	for (int i = 0; i < stateLen-1; i += amtValues) {
		Spring edgeSpring(i, i+amtValues, ks, kd, restLength, SpringType::EDGE);
		springs.push_back(edgeSpring);
	}
	// bend springs
	for (int j = 0; j < stateLen-amtValues; j += amtValues) {
		Spring bendSpring(j, j+(amtValues*2), ks, kd, restLength, SpringType::BEND);
		springs.push_back(bendSpring);
	}
}

void applyExternalForces() {
	const static Eigen::Vector3f gravityForce = gravity * mass;
	for (int i = 0; i < stateLen; i += amtValues) {
		// check if this vertex is not a root
		if (state[i+9] != 1.0f) {
			// apply gravity force
			state[i+6] = gravityForce.x();
			state[i+7] = gravityForce.y();
			state[i+8] = gravityForce.z();
		}
	}
}

void applySpringForces() {
	for (Spring s : springs) {
		s.solve_spring(state, mass, amp);
	}
}

void debugState() {
	std::cout << std::endl;
	std::cout << "at timestep " << t << ": " << std::endl;
	std::cout << std::endl;
	std::cout << "-----" << std::endl;
	for (int i = 0; i < stateLen; i += amtValues) {
		std::cout << "pt " << i/amtValues << ": " << std::endl;
		std::cout << "pos: " << state[i] << " " << state[i+1] << " " << state[i+2] << std::endl;
		std::cout << "vel: " << state[i+3] << " " << state[i+4] << " " << state[i+5] << std::endl;
		std::cout << "force: " << state[i+6] << " " << state[i+7] << " " << state[i+8] << std::endl;
		if (state[i+9] == 1.0f) std::cout << "this pt is the root" << std::endl;
		std::cout << "-----" << std::endl;
	}
	std::cout << std::endl;
}

void debugSpring(Spring s) {
	std::cout << "Spring goes from pt at starting index " << s.get_a_idx() << " to " << s.get_b_idx() << std::endl;
	std::cout << "----" << std::endl;
}

void integrate() {
	// Euler integration
	for (int i = 0; i < stateLen; i += amtValues) {
		// get acceleration
		Eigen::Vector3f acceleration(state[i+6],state[i+7],state[1+8]);
		acceleration = acceleration / mass;
		// calculate velocity
		state[i+3] += acceleration.x() * h;
		state[i+4] += acceleration.y() * h;
		state[i+5] += acceleration.z() * h;
		// calculate position
		state[i] += state[i+3] * h;
		state[i+1] += state[i+4] * h;
		state[i+2] += state[i+5] * h;
	}
}

int main( int argc, char *argv[] ) {

	// get parameters from command line
	if (argc > 1 && argc <= 6) {
		mass = std::stof(argv[1]);
		ks = std::stof(argv[2]);
		kd = std::stof(argv[3]);
		restLength = std::stof(argv[4]);
		amp = std::stof(argv[5]);
		// TODO: add h and startFrame/endFrame
		// maybe starting velocity?
	}

	// pre-process
	IO * io = new IO();
	init(io);
	initSprings();

	const std::string outfolder = "out_data/";
	std::string filepath;

	if (SIM) { // TODO: remove (for debugging)
		// simulation loop
		int frameCount = 0;
		while (frameCount < frameMax) {

			// if t is an output frame then	
			if (n % displayCheck == 0) {
				// output vertex data
				++frameCount;
				filepath = outfolder + "frame" + std::to_string(frameCount) + ".txt";
				io->write_data(filepath.c_str(), state, stateLen, amtValues);
			}

			// timestep physics logic
			applyExternalForces();
			applySpringForces();
			integrate();

			++n;
			t = n * h;
		}

		debugState();
		std::cout << "total frames: " << frameCount << std::endl;
		std::cout << "total iterations: " << n << std::endl;
	}

	return 0;
}
