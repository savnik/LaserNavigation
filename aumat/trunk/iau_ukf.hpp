/*
 * iau_ukf.hpp
 *
 *  Created on: Nov 17, 2010
 *      Author: enis
 */

#ifndef IAU_UKF_HPP_
#define IAU_UKF_HPP_

#include <iostream>
#include "smartmat.hpp"

class iau_ukf {

template <int statedim, int measnoisedim, int measdim, class auxclass>
class standardwrapper {
public:
	smartmat<measdim,1> (*measeq)(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, auxclass auxin);
	standardwrapper(smartmat<measdim,1> measeq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, auxclass auxin)):measeq(measeq) {}
	smartmat<measdim,1> measerreq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin) {
		return meas-measeq(state,noise,auxin);
	}
};

template <int statedim, int measnoisedim, int measdim, class auxclass>
class errwrapper {
public:
	smartmat<measdim,1> (*measerroreq)(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin);
	errwrapper(smartmat<measdim,1> measerreq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin)):measerroreq(measerreq) {}
	smartmat<measdim,1> measerreq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin) {
		return measerroreq(state,noise,meas,auxin);
	}
};

public:

template <int statedim, int procnoisedim, int inputdim, class host>
static void predict(smartmat<statedim,1> & stateest, smartmat<statedim,statedim> & statecov,
		const smartmat<inputdim,1> input, const smartmat<procnoisedim,procnoisedim> noisecov, host & hostobj,
		smartmat<statedim,1> (host::*statetrans)(smartmat<statedim,1> oldstate, smartmat<procnoisedim,1> noise, smartmat<inputdim,1> input) ) {

	const int L = statedim+procnoisedim, kappa = 1, beta=2;
	const double alpha = 1e-3;
	const double lambda = alpha*alpha*(L+kappa)-L;
	const double gamma = lambda + L;


	smartmat<statedim,1> statepoints[2*L+1];

	smartmat<statedim,statedim> statedevs = smartmat<statedim,statedim>(gamma*statecov).chol();
	smartmat<procnoisedim,procnoisedim> noisedevs = smartmat<procnoisedim,procnoisedim>(gamma*noisecov).chol();
	smartmat<procnoisedim,1> noiseest; noiseest.setValues(0);

	statepoints[0]=(hostobj.*statetrans)(stateest,noiseest,input);

	for(int i=0; i<statedim; i++) {
		smartmat<statedim,1> dev(statedevs,0,i);
		statepoints[i+1] = (hostobj.*statetrans)(stateest+dev,noiseest,input);
		statepoints[i+L+1] = (hostobj.*statetrans)(stateest-dev,noiseest,input);
	}

	for(int i=0; i<procnoisedim; i++) {
		smartmat<procnoisedim,1> dev(noisedevs,0,i);
		statepoints[i+statedim+1] = (hostobj.*statetrans)(stateest,noiseest+dev,input);
		statepoints[i+L+statedim+1] = (hostobj.*statetrans)(stateest,noiseest-dev,input);
	}

	const double Ws0 = lambda/(L+lambda);
	const double Wsi = 1/(2*(L+lambda));
	const double Wc0 = Ws0+(1-alpha*alpha+beta);
	const double Wci = Wsi;

	stateest =  Ws0 * statepoints[0];

	for(int i=1; i<2*L+1; i++ ) {
		stateest += Wsi*statepoints[i];
	}

	statecov = Wc0 * (statepoints[0]-stateest) * smartmat<statedim,1>(statepoints[0]-stateest).t();

	for(int i=1; i<2*L+1; i++ ) {
		statecov += Wci * (statepoints[i]-stateest) * smartmat<statedim,1>(statepoints[i]-stateest).t();
	}

}

template <int statedim, int measnoisedim, int measdim, class auxclass, class host>
static void update(smartmat<statedim,1> & stateest, smartmat<statedim,statedim> & statecov,
		const smartmat<measdim,1> meas, const smartmat<measnoisedim,measnoisedim> noisecov, auxclass auxin, host & hostobj,
		smartmat<measdim,1> (host::*measerreq)(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin)) {

	const int L = statedim+measnoisedim, kappa = 1, beta=2;
	const double alpha = 1e-3;
	const double lambda = alpha*alpha*(L+kappa)-L;
	const double gamma = lambda + L;


	smartmat<measdim,1> measerrpoints[2*L+1];

	smartmat<statedim,statedim> statedevs = smartmat<statedim,statedim>(gamma*statecov).chol();
	smartmat<measnoisedim,measnoisedim> noisedevs = smartmat<measnoisedim,measnoisedim>(gamma*noisecov).chol();
	smartmat<measnoisedim,1> noiseest; noiseest.setValues(0);

	measerrpoints[0]=(hostobj.*measerreq)(stateest,noiseest,meas, auxin);

	for(int i=0; i<statedim; i++) {
		smartmat<statedim,1> dev(statedevs,0,i);
		measerrpoints[i+1] = (hostobj.*measerreq)(stateest+dev,noiseest, meas,auxin);
		measerrpoints[i+L+1] = (hostobj.*measerreq)(stateest-dev,noiseest, meas,auxin);
	}

	for(int i=0; i<measnoisedim; i++) {
		smartmat<measnoisedim,1> dev(noisedevs,0,i);
		measerrpoints[i+statedim+1] = (hostobj.*measerreq)(stateest,noiseest+dev, meas, auxin);
		measerrpoints[i+L+statedim+1] = (hostobj.*measerreq)(stateest,noiseest-dev, meas, auxin);
	}

	const double Ws0 = lambda/(L+lambda);
	const double Wsi = 1/(2*(L+lambda));
	const double Wc0 = Ws0+(1-alpha*alpha+beta);
	const double Wci = Wsi;

	smartmat<measdim,1> measerrest =  Ws0 * measerrpoints[0];

	for(int i=1; i<2*L+1; i++ ) {
		measerrest += Wsi*measerrpoints[i];
	}

	smartmat<measdim,measdim> measerrcov = Wc0 * (measerrest-measerrpoints[0]) * smartmat<measdim,1>(measerrest-measerrpoints[0]).t();

	for(int i=1; i<2*L+1; i++ ) {
		measerrcov += Wci * (measerrest-measerrpoints[i]) * smartmat<measdim,1>(measerrest-measerrpoints[i]).t();
	}

	smartmat<statedim,measdim> statemeascov(0); // The first element of the sum is 0
	for(int i=0; i<statedim; i++) {
		smartmat<statedim,1> dev(statedevs,0,i);
		statemeascov += Wci * (dev) * smartmat<measdim,1>(measerrest-measerrpoints[i+1]).t();
		statemeascov += -Wci * (dev) * smartmat<measdim,1>(measerrest-measerrpoints[i+L+1]).t(); // Only the sigma points where the states deviate will contribute
	}

	smartmat<statedim,measdim> K = statemeascov*measerrcov.invgauss();
	stateest+= K*measerrest;
	statecov-= K*measerrcov*K.t();

}

template <int statedim, int measnoisedim, int measdim, class auxclass>
static void update(smartmat<statedim,1> & stateest, smartmat<statedim,statedim> & statecov,
		const smartmat<measdim,1> meas, const smartmat<measnoisedim,measnoisedim> noisecov, auxclass auxin,
		smartmat<measdim,1> measeq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, auxclass auxin) ) {
	standardwrapper<statedim,measnoisedim,measdim,auxclass> wrapper = standardwrapper<statedim,measnoisedim,measdim,auxclass>(measeq);
	update(stateest,statecov,meas,noisecov,auxin,wrapper,
			&standardwrapper<statedim,measnoisedim,measdim,auxclass>::measerreq);

}

template <int statedim, int measnoisedim, int measdim, class auxclass>
static void update(smartmat<statedim,1> & stateest, smartmat<statedim,statedim> & statecov,
		const smartmat<measdim,1> meas, const smartmat<measnoisedim,measnoisedim> noisecov, auxclass auxin,
		smartmat<measdim,1> measerreq(smartmat<statedim,1> state, smartmat<measnoisedim,1> noise, smartmat<measdim,1> meas, auxclass auxin) ) {

	update(stateest,statecov,meas,noisecov,auxin,errwrapper<statedim,measnoisedim,measdim,auxclass>(measerreq),
			&errwrapper<statedim,measnoisedim,measdim,auxclass>::measerreq);
}

template<int indim, int outdim, class auxclass, class host>
static void unscentedtransform(smartmat<indim,1> & inest, smartmat<indim,indim> & incov, smartmat<outdim,1> &outest, smartmat<outdim,outdim> &outcov,
		auxclass auxin, host & hostobj, smartmat<outdim,1> (host::*transformation)(smartmat<indim,1>,auxclass)) {
	const int L = indim, kappa = 1, beta=2;
	const double alpha = 1e-3;
	const double lambda = alpha*alpha*(L+kappa)-L;
	const double gamma = lambda + L;


	smartmat<outdim,1> sigmapoints[2*L+1];

	smartmat<indim,indim> devs = smartmat<indim,indim>(gamma*incov).chol();

	sigmapoints[0]=(hostobj.*transformation)(inest,auxin);

	for(int i=0; i<L; i++) {
		smartmat<indim,1> dev(devs,0,i);
		sigmapoints[i+1] = (hostobj.*transformation)(inest+dev,auxin);
		sigmapoints[i+L+1] = (hostobj.*transformation)(inest-dev,auxin);
	}

	const double Ws0 = lambda/(L+lambda);
	const double Wsi = 1/(2*(L+lambda));
	const double Wc0 = Ws0+(1-alpha*alpha+beta);
	const double Wci = Wsi;

	outest =  Ws0 * sigmapoints[0];

	for(int i=1; i<2*L+1; i++ ) {
		outest += Wsi*sigmapoints[i];
	}

	outcov = Wc0 * (sigmapoints[0]-outest) * smartmat<outdim,1>(sigmapoints[0]-outest).t();

	for(int i=1; i<2*L+1; i++ ) {
		outcov += Wci * (sigmapoints[i]-outest) * smartmat<outdim,1>(sigmapoints[i]-outest).t();
	}

}

template<int indim, int outdim, class auxclass>
class UTfunctionWrapper {
public:
	smartmat<outdim,1> (*transformation)(smartmat<indim,1>,auxclass);
	UTfunctionWrapper(smartmat<outdim,1> (*transformation)(smartmat<indim,1>,auxclass)): transformation(transformation) {}
	smartmat<outdim,1> apply(smartmat<indim,1> in ,auxclass auxin) {
		return transformation(in,auxin);
	}
};

template<int indim, int outdim, class auxclass>
static void unscentedtransform(smartmat<indim,1> & inest, smartmat<indim,indim> & incov, smartmat<outdim,1> &outest, smartmat<outdim,outdim> &outcov,
		auxclass auxin, smartmat<outdim,1> (*transformation)(smartmat<indim,1>,auxclass)) {
	UTfunctionWrapper<indim,outdim,auxclass> wrapper = UTfunctionWrapper<indim,outdim,auxclass>(transformation);
	unscentedtransform(inest,incov,outest,outcov,auxin,wrapper,
			& UTfunctionWrapper<indim,outdim,auxclass>::apply);
}


};

#endif /* IAU_UKF_HPP_ */
