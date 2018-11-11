#include <cmath>
class linear_scale {
public:
	linear_scale(float _min, float _max, float _out_min, float _out_max) {
		calibrate(_min, _max, _out_max, _out_min);
	}

	linear_scale(int _min, int _max, int _out_min, int _out_max) {
		calibrate((float)_min, (float)_max, (float)_out_max, (float)_out_min);
	}
	linear_scale(void);

	void calibrate(float _min, float _max, float _out_max, float _out_min) {
		min = _min;
		max = _max;
		out_max = _out_max;
		out_min = _out_min;

		m = (out_max - out_min) / (max - min);
		c = out_min - m * min;
	}

	float scale(float value) {
		if (value < min)
			return out_min;

		if (value > max)
			return out_max;

		return m * value + c;

	}

	int int_scale(int value) {
		return std::round(scale((float) value));
	}

private:
	float min, max, out_max, out_min, m, c;

};
