class thermostat {
public:
	thermostat(float turn_on, float turn_off) {
//		Turns on at turns on, then turns off at turn_off
//		Turns off < turns on

		_turn_on = turn_on;
		_turn_off = turn_off;

		status = 0;
		status_old = 0;
	}

	bool update(float value) {
		if (value >= _turn_on) {
			status = true;
			return true;
		} else if (value <= _turn_off) {
			status = false;
			return false;
		} else {
			return status;
		}
	}

	bool getStatus() {
		return status;
	}

	bool checkToggle(float value) {
		update(value);

		if (status_old != status) {
			status_old = status;
			return true;
		} else
			return false;
	}

private:
	float _turn_on, _turn_off, status, status_old;

};
