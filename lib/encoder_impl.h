/*
 * Copyright (C) 2014 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef INCLUDED_RDS_ENCODER_IMPL_H
#define INCLUDED_RDS_ENCODER_IMPL_H

#include <rds/encoder.h>
#include <gnuradio/thread/thread.h>
#include <chrono>

namespace gr {
namespace rds {

class encoder_impl : public encoder
{
public:
	encoder_impl(unsigned char pty_locale, int pty, bool ms,
				 bool mono_stereo, bool artificial_head, bool compressed, bool static_pty,
				 std::string ps, double af1, bool tp,
				 bool ta, int pi_country_code, int pi_coverage_area,
				 int pi_reference_number, int extended_country_code, std::string radiotext,
				 bool log, bool debug);

	virtual void update_ps(std::string ps);

private:
	~encoder_impl();

	int work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);

	unsigned int  infoword[4];
	unsigned int  checkword[4];
	unsigned int  block[4];
	unsigned char **buffer;
	unsigned char pty_locale;

	bool          log;
	bool          debug;

	// FIXME make this a struct (or a class)
	unsigned int PTY;
	unsigned char radiotext[64];
	unsigned char PS[8];
	bool TA;
	bool TP;
	bool MS;
	bool STEREO;
	bool AH;
	bool CMP;
	bool stPTY;
	unsigned int PI;
	double AF1;
	unsigned int ECC;

	int DP;
	int extent;
	int event;
	int location;
	gr::thread::mutex d_mutex;

/* each type 0 group contains 2 out of 8 PS characters;
 * this is used to count 0..3 and send all PS characters */
	int d_g0_counter;
/* each type 2A group contains 4 out of 64 RadioText characters;
 * each type 2B group contains 2 out of 32 RadioText characters;
 * this is used to count 0..15 and send all RadioText characters */
	int d_g2_counter;
/* points to the current buffer being prepared, used in create_group() */
	int d_group_counter;
/* points to the current buffer being streamed, used in in work() */
	int d_current_buffer;
/* loops through the buffer, pushing out the symbols */
	int d_buffer_bit_counter;
	int groups[32];
/* nbuffers might be != ngroups, e.g. group 0A needs 4 buffers */
	int nbuffers;

	double target_rate;
	double max_buffer_time;
	unsigned int minutes;
	std::chrono::system_clock::time_point system_clock;
	std::optional<std::chrono::steady_clock::time_point> previous_work;

// Functions
	void rebuild(bool lock = true);
	void set_mono_stereo(bool mono_stereo);
	void set_artificial_head(bool artificial_head);
	void set_compressed(bool compressed);
	void set_static_pty(bool static_pty);
	void set_ms(bool ms);
	void set_tp(bool tp);
	void set_ta(bool ta);
	void set_af1(double af1);
	void set_pty(unsigned int pty);
	void set_pi(unsigned int pty);
	void set_extended_country_code(unsigned int extended_country_code);
	void set_radiotext(std::string text);
	void set_ps(std::string ps);

	void count_groups();
	void create_group(const int, const bool);
	void prepare_group0(const bool);
	void prepare_group1a();
	void prepare_group2(const bool);
	void prepare_group3a();
	void prepare_group4a();
	void prepare_group8a();
	void prepare_group11a();
	void prepare_buffer(int);
	unsigned int encode_af(double);
	unsigned int calc_syndrome(unsigned long, unsigned char);
	void rds_in(pmt::pmt_t msg);
};

} /* namespace rds */
} /* namespace gr */

#endif /* INCLUDED_RDS_ENCODER_IMPL_H */
