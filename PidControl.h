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

class GainChannel  {
public:
	double gain = 0.0;
	double maxg = 0.0;
	double limitToMax(double v) { 
		if (maxg <= 0)
			return v;
			return min(maxg, max(-maxg, v));
	}
	double getCorrection(double v) {
		double c = v; 
		if (maxg > 0) { 
			if (c < -maxg) c = -maxg;
			if (c > maxg) c = maxg;
		}
		return c;
	}	
};

class GainControl {
public:
	GainChannel p, i, d, j, l;
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
    PID err;
    PID gain; 
    double finalGain = 1.0;
    
    // these values are set in reset() method
    double i;
    RunningLeastSquares histError, histMeasurement;
	   
    long starttime = 0;
	double corr = 0;
 
	void reset() {
		histError.reset();
		histMeasurement.reset();
        starttime = 0;
        corr = 0;
    }
    
    PidControl(int histSize, String name = "") : histError(histSize), histMeasurement(histSize) { 
    	reset();
    	description = name;
    }
    
    void rebase() {
    	histError.rebaseX();
    	histMeasurement.rebaseX();
    }
    
    double lastVal, drms;
    int count = 0;
    double add(double error, double measurement, double time) {
		if (count++ % 2000 == 0) 
			rebase();
        lastVal = error;
        
		histError.add(time, error);
		histMeasurement.add(time, measurement);
		i += error;
        
        err.p = gain.p * histError.predict(time);
 	    err.i = gain.i * i;
 	    err.d = gain.d * histMeasurement.slope(); // Derviative on Measurment
 	    //err.d = gain.d * histError.slope(); // Derviative on Error
		drms = histMeasurement.rmsError();
 	           
	    corr = -(err.p + err.i + err.d) * finalGain;
	    return corr;
    }
};
 
