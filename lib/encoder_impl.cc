/*
 * Copyright (C) 2014, 2016 Bastian Bloessl <bloessl@ccs-labs.org>
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define dout debug && std::cout
#define lout log && std::cout

#include "encoder_impl.h"
#include "constants.h"
#include <gnuradio/io_signature.h>
#include <boost/spirit/include/qi.hpp>
#include <thread>
#include <cmath>
#include <ctime>
#include <iomanip>

using namespace gr::rds;

encoder_impl::encoder_impl (unsigned char pty_locale, int pty, bool ms,
		bool mono_stereo, bool artificial_head, bool compressed, bool static_pty,
		std::string ps, double af1, bool tp,
		bool ta, int pi_country_code, int pi_coverage_area,
		int pi_reference_number, int extended_country_code, std::string radiotext,
		bool log, bool debug)
	: gr::sync_block ("gr_rds_encoder",
			gr::io_signature::make (0, 0, 0),
			gr::io_signature::make (1, 1, sizeof(unsigned char))),
	log(log),
	debug(debug),
	pty_locale(pty_locale)
{
	message_port_register_in(pmt::mp("rds in"));
	set_msg_handler(pmt::mp("rds in"), [this](pmt::pmt_t msg) { this->rds_in(msg); });

	std::memset(infoword,    0, sizeof(infoword));
	std::memset(checkword,   0, sizeof(checkword));
	std::memset(groups,      0, sizeof(groups));

	nbuffers             = 0;
	d_g0_counter         = 0;
	d_g2_counter         = 0;
	d_group_counter      = 0;
	d_current_buffer     = 0;
	d_buffer_bit_counter = 0;

	target_rate          = 1187.5;
	max_buffer_time      = 0.25;
	minutes              = 0;

	system_clock         = std::chrono::system_clock::now();

	set_mono_stereo(mono_stereo);                     // DI: mono/stereo (1=stereo)
	set_artificial_head(artificial_head);             // DI: artificial head
	set_compressed(compressed);                       // DI: compressed
	set_static_pty(static_pty);                       // DI: static PTY
	set_ms(ms);                                       // music/speech switch (1=music)
	set_af1(af1);                                     // alternate frequency 1
	set_tp(tp);                                       // traffic programm
	set_ta(ta);                                       // traffic announcement
	set_pty(pty);                                     // programm type
	set_pi((pi_country_code & 0xF) << 12 |
			(pi_coverage_area & 0xF) << 8 |
			(pi_reference_number));
	set_extended_country_code(extended_country_code); // extended country code

	set_radiotext(std::string(radiotext));
	set_ps(ps);

	DP                   = 3;
	extent               = 2;
	event                = 1340;
	location             = 11023;

	// which groups are set
	groups[ 0] = 1; // basic tuning and switching
	groups[ 1] = 1; // Extended Country Code
	groups[ 2] = 1; // radio text
	groups[ 4] = 1; // clock time

	rebuild();
}

encoder_impl::~encoder_impl() {
	free(buffer);
}

void encoder_impl::rebuild(bool lock) {
	if (lock) {
		gr::thread::scoped_lock lock(d_mutex);
	}

	count_groups();
	d_group_counter = 0;

	// allocate memory for nbuffers buffers of 104 unsigned chars each
	buffer = (unsigned char **)malloc(nbuffers * sizeof(unsigned char *));
	for(int i = 0; i < nbuffers; i++) {
		buffer[i] = (unsigned char *)malloc(104 * sizeof(unsigned char));
		for(int j = 0; j < 104; j++) buffer[i][j] = 0;
	}

	// prepare each of the groups
	for(int i = 0; i < 32; i++) {
		if(groups[i] == 1) {
			create_group(i % 16, (i < 16) ? false : true);
			if(i % 16 == 0)  // if group is type 0, call 3 more times
				for(int j = 0; j < 3; j++) create_group(i % 16, (i < 16) ? false : true);
			if(i % 16 == 2) // if group type is 2, call 15 more times
				for(int j = 0; j < 15; j++) create_group(i % 16, (i < 16) ? false : true);
			if(i % 16 == 3)  // if group is type 3, call 1 more times
				create_group(i % 16, (i < 16) ? false : true);
		}
	}

	dout << "nbuffers: " << nbuffers << std::endl;
}

void encoder_impl::rds_in(pmt::pmt_t msg) {
	if(!pmt::is_pair(msg)) {
		return;
	}

	using boost::spirit::qi::phrase_parse;
	using boost::spirit::qi::lexeme;
	using boost::spirit::qi::char_;
	using boost::spirit::qi::hex;
	using boost::spirit::qi::int_;
	using boost::spirit::qi::uint_;
	using boost::spirit::qi::bool_;
	using boost::spirit::qi::double_;
	using boost::spirit::qi::space;
	using boost::spirit::qi::blank;
	using boost::spirit::qi::lit;

	int msg_len = pmt::blob_length(pmt::cdr(msg));
	std::string in = std::string((char*)pmt::blob_data(pmt::cdr(msg)), msg_len);
	dout << "input string: " << in << "   length: " << in.size() << std::endl;

	unsigned int ui1;
	std::string s1;
	bool b1;
	double d1;

	// pty
	if(phrase_parse(in.begin(), in.end(),
			"pty" >> (("0x" >> hex) | uint_), space, ui1)) {
		lout << "set pty: " << ui1 << std::endl;
		set_pty(ui1);

	// radio text
	} else if(phrase_parse(in.begin(), in.end(),
			"text" >> lexeme[+(char_ - '\n')] >> -lit("\n"),
			space, s1)) {
		lout << "text: " << s1 << std::endl;
		set_radiotext(s1);

	// ps
	} else if(phrase_parse(in.begin(), in.end(),
			"ps" >> lexeme[+(char_ - '\n')] >> -lit("\n"),
			space, s1)) {
		lout << "ps: " << s1 << std::endl;
		set_ps(s1);

	// ta
	} else if(phrase_parse(in.begin(), in.end(),
			"ta" >> bool_, space, b1)) {
		lout << "ta: " << b1 << std::endl;
		set_ta(b1);

	// tp
	} else if(phrase_parse(in.begin(), in.end(),
			"tp" >> bool_, space, b1)) {
		lout << "tp: " << b1 << std::endl;
		set_tp(b1);

	// MS
	} else if(phrase_parse(in.begin(), in.end(),
			"ms" >> bool_, space, b1)) {
		lout << "ms: " << b1 << std::endl;
		set_ms(b1);

	// PI
	} else if(phrase_parse(in.begin(), in.end(),
			"pi" >> lit("0x") >> hex, space, ui1)) {
		lout << "set pi: " << ui1 << std::endl;
		set_pi(ui1);

	// extended country code
	} else if(phrase_parse(in.begin(), in.end(),
			"ecc" >> (("0x" >> hex) | uint_), space, ui1)) {
		lout << "extended_country_code: " << ui1 << std::endl;
		set_extended_country_code(ui1);

	// AF1
	} else if(phrase_parse(in.begin(), in.end(),
			"af1" >> double_, space, d1)) {
		lout << "set af1: " << d1 << std::endl;
		set_af1(d1);

	// DI: mono/stereo
	} else if(phrase_parse(in.begin(), in.end(),
				"mono_stereo" >> bool_, space, b1)) {
		lout << "mono_stereo: " << b1 << std::endl;
		set_mono_stereo(b1);

	// DI: artificial head
	} else if(phrase_parse(in.begin(), in.end(),
				"artificial_head" >> bool_, space, b1)) {
		lout << "artificial_head: " << b1 << std::endl;
		set_artificial_head(b1);

	// DI: compressed
	} else if(phrase_parse(in.begin(), in.end(),
				"compressed" >> bool_, space, b1)) {
		lout << "compressed: " << b1 << std::endl;
		set_compressed(b1);

	// DI: static PTY
	} else if(phrase_parse(in.begin(), in.end(),
				"static_pty" >> bool_, space, b1)) {
		lout << "static_pty: " << b1 << std::endl;
		set_static_pty(b1);

	// no match / unkonwn command
	} else {
		lout << "not understood" << std::endl;
	}

	rebuild();
}

// DI: mono/stereo
void encoder_impl::set_mono_stereo(bool mono_stereo) {
	STEREO = mono_stereo;

	lout << "setting DI (mono/stereo) to " << std::boolalpha << STEREO << std::dec << " (";
	if (STEREO) lout << "stereo)" << std::endl;
	else lout << "mono)" << std::endl;
}

// DI: artificial head
void encoder_impl::set_artificial_head(bool artificial_head) {
	AH = artificial_head;

	lout << "setting DI (artificial head) to " << std::boolalpha << AH << std::dec << std::endl;
}

// DI: compressed
void encoder_impl::set_compressed(bool compressed) {
	CMP = compressed;

	lout << "setting DI (compressed) to " << std::boolalpha << CMP << std::dec << std::endl;
}

// DI: static PTY
void encoder_impl::set_static_pty(bool static_pty) {
	stPTY = static_pty;

	lout << "setting DI (static PTY) to " << std::boolalpha << stPTY << std::dec << std::endl;
}

// music/speech
void encoder_impl::set_ms(bool ms) {
	MS = ms;

	lout << "setting Music/Speech to " << std::boolalpha << MS << std::dec << " (";
	if (MS) lout << "music)" << std::endl;
	else lout << "speech)" << std::endl;
}

// alternate frequency
void encoder_impl::set_af1(double af1) {
	AF1 = af1;

	lout << "af1 set to " << AF1 / 1000000 << std::endl;
}

// traffic program indication
void encoder_impl::set_tp(bool tp) {
	TP = tp;

	lout << "tp set to " << std::boolalpha << TP << std::dec << std::endl;
}

// traffic announcment
void encoder_impl::set_ta(bool ta) {
	TA = ta;

	lout << "ta set to " << std::boolalpha << TA << std::dec << std::endl;

	groups[ 3] = bool(TA);
	groups[ 8] = bool(TA);
	groups[11] = bool(TA);
}

// program type
void encoder_impl::set_pty(unsigned int pty) {
	if(pty > 31) {
		std::cout << "warning: ignoring invalid pty: " << pty << std::endl;
	} else {
		PTY = pty;
		lout << "setting pty to " << PTY << " (" << pty_table[PTY][pty_locale] << ")" << std::endl;
	}
}

// program identification
void encoder_impl::set_pi(unsigned int pi) {
	if(pi > 0xFFFF) {
		std::cout << "warning: ignoring invalid pi: " << pi << std::endl;
	} else {
		PI = pi;
		lout << "setting pi to " << std::uppercase << std::hex << PI << std::dec << std::endl;
		if(PI & 0xF000) {
			int pi_country_id = ((PI & 0xF000) >> 12) - 1;

			lout << "    country code " << pi_country_codes[pi_country_id][0] << " ";
			lout << pi_country_codes[pi_country_id][1] << " | ";
			lout << pi_country_codes[pi_country_id][2] << " | ";
			lout << pi_country_codes[pi_country_id][3] << " | ";
			lout << pi_country_codes[pi_country_id][4] << std::endl;
		} else {
			lout << "    country code 0 (incorrect)" << std::endl;
		}
		lout << "    coverage area " << coverage_area_codes[(PI & 0xF00) >> 8] << std::endl;
		lout << "    program reference number " << (PI & 0xFF) << std::endl;
	}
}

// extended country code
void encoder_impl::set_extended_country_code(unsigned int extended_country_code) {
	if(extended_country_code > 4) {
		std::cout << "warning: ignoring invalid extended_country_code: " << extended_country_code << std::endl;
	} else {
		ECC = extended_country_code;
		lout << "setting extended_country_code to " << std::uppercase << std::hex << (0xE0 | ECC) << std::dec << std::endl;

		if(PI & 0xF000) {
			int pi_country_id = ((PI & 0xF000) >> 12) - 1;
			lout << "    extended country code " << pi_country_codes[pi_country_id][ECC] << std::endl;
		}
	}
}

// radiotext
void encoder_impl::set_radiotext(std::string text) {
	size_t len = std::min(sizeof(radiotext), text.length());

	std::memset(radiotext, ' ', sizeof(radiotext));
	std::memcpy(radiotext, text.c_str(), len);

	lout << "radiotext set to \"" << std::string((char *)radiotext, len) << "\"" << std::endl;
}

// program service name
void encoder_impl::set_ps(std::string ps) {
	size_t len = std::min(sizeof(PS), ps.length());

	std::memset(PS, ' ', sizeof(PS));
	std::memcpy(PS, ps.c_str(), len);

	lout << "PS set to \"" << std::string((char *)PS, len) << "\"" << std::endl;
}

// separate method for gnuradio callback to avoid duplicate rebuild
void encoder_impl::update_ps(std::string ps) {
	set_ps(ps);
	rebuild();
}

/* see Annex B, page 64 of the standard */
unsigned int encoder_impl::calc_syndrome(unsigned long message, unsigned char mlen) {
	unsigned long reg = 0;
	unsigned int i;
	const unsigned long poly = 0x5B9;
	const unsigned char plen = 10;

	for (i = mlen; i > 0; i--)  {
		reg = (reg << 1) | ((message >> (i - 1)) & 0x01);
		if (reg & (1 << plen)) reg = reg ^ poly;
	}
	for (i = plen; i > 0; i--) {
		reg = reg << 1;
		if (reg & (1 << plen)) reg = reg ^ poly;
	}
	return reg & ((1 << plen) - 1);
}

/* see page 41 in the standard; this is an implementation of AF method A
 * FIXME need to add code that declares the number of AF to follow... */
unsigned int encoder_impl::encode_af(const double af) {
	dout << "encoding af: " << af << std::endl;
	unsigned int af_code = 0;
	if(( af >= 87.6) && (af <= 107.9))
		af_code = nearbyint((af - 87.5) * 10);
	else if((af >= 153) && (af <= 279))
		af_code = nearbyint((af - 144) / 9);
	else if((af >= 531) && (af <= 1602))
		af_code = nearbyint((af - 531) / 9 + 16);
	else
		std::cout << "warning: invalid alternate frequency: " << af << std::endl;
	return af_code;
}

/* count and print present groups */
void encoder_impl::count_groups(void) {
	int ngroups = 0;
	nbuffers = 0;
	dout << "groups present: ";
	for(int i = 0; i < 32; i++) {
		if(groups[i] == 1) {
			ngroups++;
			dout << i % 16 << ((i < 16) ? 'A' : 'B') << " ";
			if(i % 16 == 0)  // group 0
				nbuffers += 4;
			else if(i % 16 == 2)  // group 2
				nbuffers += 16;
			else if(i % 16 == 3)
				nbuffers += 2;
			else
				nbuffers++;
		}
	}
	dout << "(" << ngroups << " groups)" << std::endl;
}

/* create the 4 infowords, according to group type.
 * then calculate checkwords and put everything in the groups */
void encoder_impl::create_group(const int group_type, const bool AB) {
	infoword[0] = PI;
	infoword[1] = (((group_type & 0xf) << 12) | (AB << 11) | (TP << 10) | (PTY << 5));

	if(group_type == 0) prepare_group0(AB);
	else if(group_type == 1) prepare_group1a();
	else if(group_type == 2) prepare_group2(AB);
	else if(group_type == 3) prepare_group3a();
	else if(group_type == 4) prepare_group4a();
	else if(group_type == 8) prepare_group8a();
	else if(group_type == 11) prepare_group11a();
	else std::cout << "warning: preparation of group " << group_type << " not yet supported" << std::endl;

	dout << "data: " << std::uppercase << std::hex << std::setfill('0') << std::setw(4) << infoword[0] << " " << infoword[1] << " " << infoword[2] << " " << infoword[3] << ", ";

	for(int i= 0; i < 4; i++) {
		checkword[i]=calc_syndrome(infoword[i], 16);
		block[i] = ((infoword[i] & 0xffff) << 10) | (checkword[i] & 0x3ff);
		// add the offset word
		if((i == 2) && AB) block[2] ^= offset_word[4];
		else block[i] ^= offset_word[i];
	}

	dout << "group: " << block[0] << " " << block[1] << " " << block[2] << " " << block[3] << std::dec << std::endl;

	prepare_buffer(d_group_counter);
	d_group_counter++;
}

void encoder_impl::prepare_group0(const bool AB) {
	dout << "preparing group 0" << std::endl;
	infoword[1] = infoword[1] | (TA << 4) | (MS << 3);
	switch (d_g0_counter) {
		case 3:
			infoword[1] = infoword[1] | (STEREO << 2);
		break;
		case 2:
			infoword[1] = infoword[1] | (AH << 2);
		break;
		case 1:
			infoword[1] = infoword[1] | (CMP << 2);
		break;
		case 0:
			infoword[1] = infoword[1] | (!stPTY << 2);
		break;
		default:
		break;
	}
	infoword[1] = infoword[1] | (d_g0_counter & 0x3);
	if(!AB) {
		infoword[2] = (225 << 8) | // 1 AF follows
			(encode_af(AF1 / 1000000) & 0xff);
	} else {
		infoword[2] = PI;
	}
	infoword[3] = (PS[2 * d_g0_counter] << 8) | PS[2 * d_g0_counter + 1];
	d_g0_counter++;
	if(d_g0_counter > 3) d_g0_counter = 0;
}

void encoder_impl::prepare_group1a(void) {
	dout << "preparing group 1" << std::endl;
	//infoword[1] = infoword[1] | (1 << 4); // TMC in 8A
//	infoword[2] = (0x80 << 8) | 0xE0;
	infoword[2] = 0xE0 | ECC; // extended country code
	infoword[3] = 0; // time
}

void encoder_impl::prepare_group2(const bool AB) {
	dout << "preparing group 2" << std::endl;
	infoword[1] = infoword[1] | ((AB << 4) | (d_g2_counter & 0xf));
	if(!AB) {
		infoword[2] = (radiotext[d_g2_counter * 4] << 8 | radiotext[d_g2_counter * 4 + 1]);
		infoword[3] = (radiotext[d_g2_counter * 4 + 2] << 8 | radiotext[d_g2_counter * 4 + 3]);
	}
	else {
		infoword[2] = PI;
		infoword[3] = (radiotext[d_g2_counter * 2] << 8 | radiotext[d_g2_counter * 2 + 1]);
	}
	d_g2_counter++;
	d_g2_counter %= 16;
}

void encoder_impl::prepare_group3a(void) {
	dout << "preparing group 3" << std::endl;
	static int count = 0;
	if(count) {
		infoword[1] = infoword[1] | (0x31d0 & 0x1f);
		infoword[2] = 0x6280;
		infoword[3] = 0xcd46;
	} else {
		infoword[1] = infoword[1] | (0x31d0 & 0x1f);
		infoword[2] = 0x0066;
		infoword[3] = 0xcd46; // AID for TMC (Alert C)
	}
	count++;
	count = count % 2;
}

/* see page 28 and Annex G, page 81 in the standard */
void encoder_impl::prepare_group4a(void) {
	dout << "preparing group 4" << std::endl;
	time_t rightnow = std::chrono::system_clock::to_time_t(system_clock);

	dout << "localtime: " << asctime(localtime(&rightnow));

	/* we're supposed to send UTC time; the receiver should then add the
	* local timezone offset */
	tm *utc = gmtime(&rightnow);
	int m = utc->tm_min;
	int h = utc->tm_hour;
	int D = utc->tm_mday;
	int M = utc->tm_mon + 1;  // January: M=0
	int Y = utc->tm_year;
	int toffset=localtime(&rightnow)->tm_gmtoff / 60 / 60;

	dout << "toffset: " << toffset << std::endl;

	int L = ((M == 1) || (M == 2)) ? 1 : 0;
	int mjd=14956+D+int((Y-L)*365.25)+int((M+1+L*12)*30.6001);

	infoword[1]=infoword[1]|((mjd>>15)&0x3);
	infoword[2]=(((mjd>>7)&0xff)<<8)|((mjd&0x7f)<<1)|((h>>4)&0x1);
	infoword[3]=((h&0xf)<<12)|(((m>>2)&0xf)<<8)|((m&0x3)<<6)|
		((toffset>0?0:1)<<5)|((abs(toffset*2))&0x1f);

	minutes = m;
	dout << "set minutes to " << minutes << std::endl;
}

// TMC Alert-C
void encoder_impl::prepare_group8a(void) {
	dout << "preparing group 8" << std::endl;
	infoword[1] = infoword[1] | (1 << 3) | (DP & 0x7);
	infoword[2] = (1 << 15) | ((extent & 0x7) << 11) | (event & 0x7ff);
	infoword[3] = location;
}

// for now single-group only
void encoder_impl::prepare_group11a(void) {
	dout << "preparing group 11" << std::endl;
	infoword[1] = infoword[1] | (0xb1c8 & 0x1f);
	infoword[2] = 0x2038;
	infoword[3] = 0x4456;
}

void encoder_impl::prepare_buffer(int which) {
	int q=0, a=0, b=0;

	for(q = 0; q < 104; q++) {
		a = floor(q / 26);
		b = 25 - q % 26;
		buffer[which][q] = (unsigned char)(block[a] >> b) & 0x1;
	}
}

//////////////////////// WORK ////////////////////////////////////
int encoder_impl::work (int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {

	gr::thread::scoped_lock lock(d_mutex);
	unsigned char *out = (unsigned char *) output_items[0];

	long max_items;
	double long max_time_span = max_buffer_time * 1000;

	if (previous_work.has_value()) {
		system_clock = std::chrono::system_clock::now();
		time_t rightnow = std::chrono::system_clock::to_time_t(system_clock);
		tm *utc = gmtime(&rightnow);

		if (utc->tm_min != minutes) {
			dout << "changed minute, updating time" << std::endl;
			rebuild(false);
		}

		std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();
		std::chrono::duration time_span = steady_now - previous_work.value();
		double long time_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count();

		dout << "time diff " << time_span_ms << " ms" << std::endl;

		if (max_time_span > time_span_ms) {
			return 0;
		}

		max_time_span = time_span_ms;
	}

	previous_work = std::chrono::steady_clock::now();
	max_items = ceil(max_time_span / 1000 * target_rate);

	int sent_items = 0;

	for(int i = 0; i < noutput_items; i++) {
		out[i] = buffer[d_current_buffer][d_buffer_bit_counter];

		if(++d_buffer_bit_counter > 103) {
			d_buffer_bit_counter = 0;
			d_current_buffer++;
			d_current_buffer = d_current_buffer % nbuffers;
		}

		if (++sent_items > max_items) {
			break;
		}
	}

	dout << "sent items " << sent_items << std::endl;

	return sent_items;
}

encoder::sptr encoder::make (unsigned char pty_locale, int pty, bool ms,
		bool mono_stereo, bool artificial_head, bool compressed, bool static_pty,
		std::string ps, double af1, bool tp,
		bool ta, int pi_country_code, int pi_coverage_area,
		int pi_reference_number, int extended_country_code, std::string radiotext,
		bool log, bool debug) {

	return gnuradio::get_initial_sptr(
			new encoder_impl(pty_locale, pty, ms, mono_stereo, artificial_head, compressed, static_pty,
					ps, af1, tp, ta, pi_country_code, pi_coverage_area, pi_reference_number, extended_country_code,
					radiotext, log, debug));
}
