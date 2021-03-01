//This module provides a journal for logging to disk and optionally printing to a standard stream
//Author: Bryan Poling
//Copyright (c) 2018 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <string>
#include <regex>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <mutex>

//External Includes
#include "../../handycpp/Handy.hpp"

//A journal manages a file on disk and allows printing time-stamped text to the file. It can also (optionally) print to cout or cerr.
//All printing is thread-safe. The print and print_continued functions are slightly more efficient than the printf variants, so use them for plain text.
class Journal {
	public:
		Journal(std::filesystem::path LogPath, std::ostream * Stream = nullptr, bool Append = true);
		Journal(const Journal & Other) = delete;
		Journal & operator= (const Journal & Other) = delete;
		~Journal();
		
		void print(std::string const & Message); //Print a std::string to journal
		void printf(const char * fmt, ...);      //Print to journal using printf-style formatting
		
		//We also offer "continued" versions of print and printf. Consecutive calls to these functions will put text on the same
		//line (with only 1 Date/Time header in the journal) until a call to print or printf is made, which will finish off the line.
		void print_continued(std::string const & Message);
		void printf_continued(const char * fmt, ...);
		
	private:
		std::ofstream m_fileStream;
		std::ostream * m_stream;
		bool continued; //True if the last print call was print_continued of printf_continued.
		std::mutex m_mutex;
		
		std::string GetDateAndTimeString(void);
		std::string BufferNewLines(std::string S, size_t Width); //Post-pad newlines with the given number of space chars
};

inline Journal::Journal(std::filesystem::path LogPath, std::ostream * Stream, bool Append) {
	m_stream = Stream;
	if (! LogPath.empty()) {
		if (Append)
			m_fileStream.open(LogPath.string().c_str(), std::ofstream::out | std::ofstream::binary | std::ofstream::app);
		else
			m_fileStream.open(LogPath.string().c_str(), std::ofstream::out | std::ofstream::binary);
	}
	if (! m_fileStream.is_open()) {
		std::cerr << "Warning in Journal::Journal(): Could not open journal file for writing. File journaling will be disabled.\r\n";
		std::cerr << "Journal file path: " << LogPath.string() << "\r\n";
		return;
	}
	else
		m_fileStream << "*******************************************   New Session   *******************************************\r\n";
	continued = false;
}

inline Journal::~Journal() {
	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_fileStream.is_open()) {
		std::string DataAndTime = this->GetDateAndTimeString();
		m_fileStream << DataAndTime + ": Closing Journal\r\n\r\n\r\n\r\n";
		m_fileStream.flush(); //Make sure we write the message now to make debugging easier
	}
	if (m_stream != nullptr) {
		*m_stream << "Closing Journal\r\n";
		m_stream->flush();
	}
}

inline void Journal::print(std::string const & Message) {
	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_fileStream.is_open()) {
		std::string DateTime = this->GetDateAndTimeString();
		std::string S = BufferNewLines(Message, DateTime.size() + 2U);
		if (continued)
			m_fileStream << S << "\r\n";
		else
			m_fileStream << DateTime + ": " + S + "\r\n";
		m_fileStream.flush(); //Make sure we write the message now to make debugging easier
	}
	if (m_stream != nullptr)
		*m_stream << Message << std::endl;
	continued = false;
}

inline void Journal::printf(const char * fmt, ...) {
	//Print formatted text to a std::string
	std::string message;
	{
		char stack_buffer[64];
		char * heap_buffer = nullptr;
		char * choosen_buffer = stack_buffer;

		va_list vl_original;
		va_list vl_working;
		va_list vl_working_2;

		va_start(vl_original, fmt);
		va_copy (vl_working,   vl_original);
		va_copy (vl_working_2, vl_original);

		size_t chars_that_would_be_printed = vsnprintf(stack_buffer, 64, fmt, vl_working);
		if (chars_that_would_be_printed >= size_t(64)) {
			unsigned heap_buffer_size = chars_that_would_be_printed+1;
			heap_buffer = new char[heap_buffer_size];
			vsnprintf(heap_buffer, heap_buffer_size, fmt, vl_working_2);
			choosen_buffer = heap_buffer;
		}

		va_end(vl_working);
		va_end(vl_working_2);
		va_end(vl_original);
		message.assign(choosen_buffer);
		if (heap_buffer)
			delete heap_buffer;
	}
	
	print(message);
}

inline void Journal::print_continued(std::string const & Message) {
	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_fileStream.is_open()) {
		std::string DateTime = this->GetDateAndTimeString();
		std::string S = BufferNewLines(Message, DateTime.size() + 2U);
		if (continued)
			m_fileStream << S;
		else
			m_fileStream << DateTime + ": " + S;
		m_fileStream.flush(); //Make sure we write the message now to make debugging easier
	}
	if (m_stream != nullptr) {
		*m_stream << Message;
		m_stream->flush();
	}
	continued = true;
}

inline void Journal::printf_continued(const char * fmt, ...) {
	//Print formatted text to a std::string
	std::string message;
	{
		char stack_buffer[64];
		char * heap_buffer = nullptr;
		char * choosen_buffer = stack_buffer;

		va_list vl_original;
		va_list vl_working;
		va_list vl_working_2;

		va_start(vl_original, fmt);
		va_copy (vl_working,   vl_original);
		va_copy (vl_working_2, vl_original);

		size_t chars_that_would_be_printed = vsnprintf(stack_buffer, 64, fmt, vl_working);
		if (chars_that_would_be_printed >= size_t(64)) {
			unsigned heap_buffer_size = chars_that_would_be_printed+1;
			heap_buffer = new char[heap_buffer_size];
			vsnprintf(heap_buffer, heap_buffer_size, fmt, vl_working_2);
			choosen_buffer = heap_buffer;
		}

		va_end(vl_working);
		va_end(vl_working_2);
		va_end(vl_original);
		message.assign(choosen_buffer);
		if (heap_buffer)
			delete heap_buffer;
	}
	
	print_continued(message);
}

inline std::string Journal::GetDateAndTimeString(void) {
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);

	std::ostringstream oss;
	oss << std::put_time(&tm, "%m/%d/%Y %H:%M:%S");
	return oss.str();
}

inline std::string Journal::BufferNewLines(std::string S, size_t Width) {
	return std::regex_replace(S, std::regex("\\r\\n"), "\r\n" + std::string(Width, ' '));
}

