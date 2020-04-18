#ifndef _RunningLeastSquares_h_
#define _RunningLeastSquares_h_
/*
 * Tips for expanding this into a general running quadratic or linear least squares regression 
 * Based on 
 * http://mathforum.org/library/drmath/view/72047.html
 * 
 *  
 * maintain sums such that  
 * Sab  is sum of X^a*Y^b for all each x and y in the data set. 
 * 
 * so when adding a point x,y with weight w
 * 
 *  S00 += 1 * w;
 *  S10 += x * w;
 *  S20 += x * x * w;
 *  S30 += x * x * x * w;
 *  S40 += x * x * x * x * w;
 *  S01 += y * w;
 *  S11 += x * y * w;
 *  S21 += x * x * y * w;
 *  S31 += x * x * x * y * w;
 *  S41 += x * x * x * x * y * w;
 * 
 *  Then the parameters a,b,c of the best fit equation y = ax^2 + bx + c
 *  are given by 
 *  
  a = (S01*S10*S30 - S11*S00*S30 - S01*S20*S20
       + S11*S10*S20 + S21*S00*S20 - S21*S10*S10)
    /(S00*S20*S40 - S10*S10*S40 - S00*S30*S30 + 2*S10*S20*S30 - S20*S20*S20)

  b = (S11*S00*S40 - S01*S10*S40 + S01*S20*S30
       - S21*S00*S30 - S11*S20*S20 + S21*S10*S20)
    /(S00*S20*S40 - S10*S10*S40 - S00*S30*S30 + 2*S10*S20*S30 - S20*S20*S20)

  c = (S01*S20*S40 - S11*S10*S40 - S01*S30*S30
       + S11*S20*S30 + S21*S10*S30 - S21*S20*S20)
    /(S00*S20*S40 - S10*S10*S40 - S00*S30*S30 + 2*S10*S20*S30 - S20*S20*S20)
 
  
  and the parameters m and b of the best fit y = mx + b are given by 
  
  m = (S00 * S11 - S10 * S01) / (S00 * S20 - S10 * S10)
  b = (S01 - m * S10) / S00 
  
  and of course the average is given by 
  
  y = S01 / S00  (sum of y's divided by count)
  
  variance and sum of errors could probably also be computed from these sums, dunno
  
 */

template <class T, int SIZE>
class RollingAverage {
	T values[SIZE];
	T sum = 0;
	int count = 0;
	int index = 0;
public:
	RollingAverage() {}
	void add(const T &v) { 
		if (count == SIZE) 
			sum -= values[index];
		else 
			count++;
		values[index++] = v;
		sum += v;
		if (index >= SIZE)
			index = 0;
	}
	T average() { 
		return count > 0 ? sum/count : 0;
	}
	T min() { 
		T m = values[0];
		for (int n = 0; n < count; n++)
			if (values[n] < m) m = values[n];
		return m;
	}
	T max() { 
		T m = values[0];
		for (int n = 0; n < count; n++)
			if (values[n] > m) m = values[n];
		return m;
	}
};


class RunningLeastSquares {
    public:
    double *xs, *ys, *weights;
    int index = 0, size = 0;
    int totalWeight = 0;
    double X = 0, Y = 0, XX = 0, XY = 0, xBase = 0;
    int count = 0;
    void clear() { X = Y = XX = XY = 0; count = 0; } 
    
    RunningLeastSquares(int s) {
        size = s;
        xs = new double[size];
        ys = new double[size];
        weights = new double[size];
    }
    ~RunningLeastSquares() {
		delete xs;
		delete ys;
		delete weights;
	}
    void add(double x, double y) { 
    	add(x, y, 1.0);
    }
    double slope() {
        if (count < 2) return 0.0;
        return (totalWeight * XY - X * Y) / (totalWeight * XX - X * X);
	}
	
	
	double intercept() { 
		if (count < 1) return 0.0;
		double b = (Y - slope() * X) / (totalWeight);
		return slope() * -xBase + b;
	}
	
    double predict(double x) { 
		//x -= xBase;
    	return slope() * x + intercept();
	}
    double err(double x, double y) {
		x -= xBase;
		return std::abs((-x * slope()) + y - intercept()) / 
                        sqrt(slope() * slope() + 1);
	}
    double rmsError() {
        double s = 0;
        for (int i = 0; i < count; i++) {
            int idx = (index + size - count + i) % size;
            double e = err(xs[idx], ys[idx]);
            s += e * e;
        }
        return sqrt(s / count);
    }
    
    void rebaseX() {
		if (count < size)
			return;
		
		double delta = xs[0] - xBase;
		xBase = xs[0];
		X = Y = XX = XY = totalWeight = 0;
		for(int n = 0; n < count; n++) {
			xs[n] -= delta;
            X += xs[n] * weights[n];
            Y += ys[n] * weights[n];
            XX += xs[n] * xs[n] * weights[n];
            XY += xs[n] * ys[n] * weights[n];
            totalWeight += weights[n];
		}
	}
    double averageY() {
    	return totalWeight > 0 ? Y / totalWeight : 0;
    }
    void add(double x, double y, double w) {
		x -= xBase;
        if (count == size) {
            X -= xs[index] * weights[index];
            Y -= ys[index] * weights[index];
            XX -= xs[index] * xs[index] * weights[index];
            XY -= xs[index] * ys[index] * weights[index];
            totalWeight -= weights[index];
        }
        
        weights[index] = w;
        xs[index] = x;
        ys[index++] = y;
        
        if (index >= size) {
            index = 0;
        }
        if (count < size) {
            count++;
        }

        X += x * w;
        Y += y * w;
        XX += x * x * w;
        XY += x * y * w;
        totalWeight += w;
    }


    void reset() {
        count = index = xBase = 0;
        X = Y = XX = XY = totalWeight = 0.0;
    }

    void removeLast() {
        count--;
        index--;
        if (index < 0) {
            index = size - 1;
        }

        X -= xs[index] * weights[index];
        Y -= ys[index] * weights[index];
        XX -= xs[index] * xs[index] * weights[index];
        XY -= xs[index] * ys[index] * weights[index];
        totalWeight -= weights[index];
    }
};
		
#endif 
