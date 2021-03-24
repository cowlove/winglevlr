class Average {
public:
	double sum = 0;
	long count = 0;
	void add(double d) { sum += d; count++; } 
	double average() { return count > 0 ? sum / count : 0;  }
};

class PID {
public:
	PID(double ap, double ai, double ad, double aj, double al) {
		p = ap;
		i = ai;
		d = ad;
		j = aj;
		l = al;
	}
	PID() { p = i = d = j = l = 0; } 
	double p, i, d, j,  l;
	String toString(String pref) { return String(""); }
	//	return String.format("%sp=%.2f, %si=%.2f, %sd=%.2f, %sj=%.2f, %sl=%.2f" +
	//			pref, p, pref, i, pref, d, pref, j, pref, l); }
};


class PidControl {
public:
    String description;
    String toString(String pref) { return String(""); } 
//    	return String.format("%se=%.2f, %sdef=%.2f, %sq=%.2f, %sdrms=%f, ", pref, corr, 
//    			pref, defaultValue.calculate(), pref, quality, pref, drms) + err.toString(pref);
//    }
    void setGains(double gp, double gi, double gd, double gj = 0, double gl = 0) { 
    	gain.p = gp;
      	gain.i = gi;
      	gain.d = gd;
      	gain.j = gj;
      	gain.l = gl;
    }
    PID err, gain, maxerr, hiGain, hiGainTrans;
    double finalGain = 1.0;
    
	float calcGain(float err, float loGain, float hiGain, float transition) {
		float c =  err * loGain;
		if (hiGain > 0 && (abs(err) > transition))
			c += (abs(err) - transition) * (hiGain - loGain) * err/abs(err);
		return c;
	}

    // these values are set in reset() method
    double i;
    RollingLeastSquares histError, histMeasurement, histCorrection;
	   
    long starttime = 0;
	double corr = 0;
 
	void reset() {
		histError.reset();
		histMeasurement.reset();
        starttime = 0;
        corr = 0;
        i = 0;
    }
    
    PidControl(int histSize, int cSize = 1, String name = "") : histError(histSize), histMeasurement(histSize), histCorrection(cSize) { 
    	reset();
    	description = name;
    }
    
    void rebase() {
    	histError.rebaseX();
    	histMeasurement.rebaseX();
    }
    
    double lastVal, lastTime = 0;
    int count = 0;
    double add(double error, double measurement, double time) {
		if (count++ % 2000 == 0) 
			rebase();
        lastVal = error;
        
        float dt = (count > 0) ? time - lastTime : 0.0;
        lastTime = time;
		histError.add(time, error);
		histMeasurement.add(time, measurement);
        err.p = calcGain(histError.predict(time), gain.p, hiGain.p, hiGainTrans.p);
        i += calcGain(histError.predict(time), gain.i, hiGain.i, hiGainTrans.i) * dt;
 	    if (maxerr.i > 0) 
			i = min(maxerr.i, max(-maxerr.i, i));
 	    err.i = i;
 	    err.l = calcGain(histCorrection.averageY(), gain.l, hiGain.l, hiGainTrans.l);
 	    err.d = calcGain(histMeasurement.slope(), gain.d, hiGain.d, hiGainTrans.d);
 	    //err.d = calcGain(histError.slope(), gain.d, hiGain.d, hiGainTrans.d);
 	    //drms = histMeasurement.rmsError();
 	           
	    corr = -(err.p + err.i + err.d + err.l) * finalGain;
	    histCorrection.add(time, corr / finalGain);
	    return corr;
    }
    
    void resetI() { i = 0; } 
};
 
