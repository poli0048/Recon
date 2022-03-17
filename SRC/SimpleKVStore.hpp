//This module provides a simple, persistent, two-file KV store for arbitrary data. Values are loaded from disk
//in a lazy fashion, but the key table is pre-loaded. This makes it fast to check if keys exist and it speeds up
//value retrieval, but it makes this store inappropriate for massive databases.
//
//Values are written immediately to a values file, but new keys are only written (along with the old keys) back to the
//keys file when the store is closed. As a result, there is only a short window of time (closing the store) when a
//write interruption (e.g. program crash) could corrupt the store. If the program crashes while the store is open
//the keys file will have the same contents as when the store was last opened. There will be some unreferenced items
//at the end of the values file, but they will be purged the next time the store is densified.
//
//This KV store type-punnes primitive types and is unaware of endianness for any data that may be encoded in values.
//Thus, KV store files may not be compatible accross different CPU architectures and platforms. This is done for
//simplicity and performance and makes the format ideal for data caches, but not really for data exchange.
//Author: Bryan Poling
//Copyright (c) 2019 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <string>
#include <unordered_map>
#include <cstdint>

//External Includes
#include "../../handycpp/Handy.hpp" //Provides std::filesystem and Handy::File

//GEMS-Core Includes
#include "Journal.h"

//There are two files in a KVStore: A Keys file and a Values file. The keys file consists of a sequence of items
//of the form: KeyLength, Key, ValueOffset, ValueLength
//KeyLength:   uint16 - implicitly limits key strings to 65,535 chars. This field cannot be 0 (empty keys are not allowed)
//Key:         String
//ValueOffset: uint64 - offset in values file where value begins
//ValueLength: uint64 - number of bytes in the value
//
//The values file simply consists of densely packed value items.

//This class is thread-safe
class SimpleKVStore {
	private:
		std::mutex Mtx;
		
		Journal & Log;
		std::filesystem::path StorePath; //Path to store (without file extension)
		Handy::File ValuesFile;
		std::unordered_map<std::string, std::tuple<uint64_t, uint64_t>> Index; //Key --> <Offset, ValueLength>
		
		inline bool Open();
		inline void Close();
		
		inline bool DensifyValuesFile();
		inline bool ReadValue(uint64_t Offset, uint64_t Size, std::vector<uint8_t> & Value);
		
		//We use a stopwatch to update internal measurements of store size periodically (if being polled).
		//This is to reduce overhead if hammering the inspection methods. We don't update these at all except in those methods.
		Handy::StopWatch m_sizeRefreshStopwatch;
		uint64_t m_numBytes;
		uint64_t m_numBytesOnDisk;
		inline void RefreshSizesIfNeeded();
	public:
		SimpleKVStore() = delete;
		SimpleKVStore(std::filesystem::path const & StorePathArg, Journal & LogRef) : Log(LogRef), StorePath(StorePathArg) { Open(); }
		~SimpleKVStore() { Close(); }
		
		bool IsOpen(void) {	std::scoped_lock slock(Mtx); return ValuesFile.IsOpen(); }
		bool Has(std::string const & Key) { std::scoped_lock slock(Mtx); return (Index.count(Key) > 0U); }
		
		inline bool Get(std::string const & Key, std::vector<uint8_t> & Value);
		inline bool Put(std::string const & Key, std::vector<uint8_t> const & Value);
		inline bool PutIfNew(std::string const & Key, std::vector<uint8_t> const & Value);
		
		void Delete(std::string const & Key) { std::scoped_lock slock(Mtx); Index.erase(Key); }
		void Clear(void) { std::scoped_lock slock(Mtx); Index.clear(); DensifyValuesFile(); }
		
		//Store Size Inspection
		uint64_t GetNumItems() { std::scoped_lock slock(Mtx); return (ValuesFile.IsOpen() ? Index.size() : 0U); }
		uint64_t GetNumBytes() { std::scoped_lock slock(Mtx); RefreshSizesIfNeeded(); return m_numBytes; }
		uint64_t GetNumBytesOnDisk() { std::scoped_lock slock(Mtx); RefreshSizesIfNeeded(); return m_numBytesOnDisk; }
};

inline bool SimpleKVStore::Open() {
	if (ValuesFile.IsOpen())
		Close();
	Index.clear();
	if (! ValuesFile.Open(StorePath + ".Values"s, false)) {
		Log.print("Error in SimpleKVStore::Open(): Could not open values file.");
		return false;
	}
	
	//Parse the Keys file
	std::vector<uint8_t> KeysBuffer;
	if (! Handy::TryReadFile(StorePath + ".Keys"s, KeysBuffer))
		Log.print("Unable to read Keys file: Starting with empty key map");
	else {
		size_t offset = 0U;
		while (offset < KeysBuffer.size()) {
			//There is data left in the buffer
			if (offset + 2U > KeysBuffer.size()) {
				Log.print("Warning: Dropping incomplete key item.");
				break;
			}
			uint16_t KeyLength = *((uint16_t *) &(KeysBuffer[offset]));
			offset += 2U;
			if (offset + KeyLength > KeysBuffer.size()) {
				Log.print("Warning: Dropping incomplete key item.");
				break;
			}
			std::string Key((char *) &(KeysBuffer[offset]), KeyLength);
			offset += KeyLength;
			if (offset + 16U > KeysBuffer.size()) {
				Log.print("Warning: Dropping incomplete key item.");
				break;
			}
			uint64_t ValueOffset = *((uint64_t *) &(KeysBuffer[offset]));
			offset += 8U;
			uint64_t ValueLength = *((uint64_t *) &(KeysBuffer[offset]));
			offset += 8U;
			
			Index[Key] = std::tuple<uint64_t,uint64_t>(ValueOffset, ValueLength);
		}
	}
	
	return true;
}

inline void SimpleKVStore::Close() {
	//If necessary, densify the values file - This must be done before writing the keys file since bad keys can be removed here
	ValuesFile.Flush();
	//DensifyValuesFile();
	std::error_code ec;
	double ValueFileSizeBytes = (double) std::filesystem::file_size(StorePath + ".Values"s, ec);
	if (ec)
		Log.print("Warning: Skipping densification because we couldn't get values file size.");
	else {
		uint64_t TotalValueBytes = 0U;
		for (auto const & kv : Index)
			TotalValueBytes += std::get<1>(kv.second);
		double WastedBytes = ValueFileSizeBytes - (double) TotalValueBytes;
		if (WastedBytes / ValueFileSizeBytes > 0.25) {
			Log.print("Wasted space in values file exceeds 25%. Attempting densification."s);
			if (DensifyValuesFile())
				Log.print("Densification succeeded.");
			else
				Log.print("Densification failed.");
		}
	}
	
	//Close the Values file
	ValuesFile.Close();
	
	//Write Keys file
	std::vector<uint8_t> KeysBuffer;
	for (auto const & kv : Index) {
		uint16_t KeySize = kv.first.size();
		uint64_t ValueOffset = std::get<0>(kv.second);
		uint64_t ValueLength = std::get<1>(kv.second);
		size_t BlockSize = 18U + KeySize;
		KeysBuffer.resize(KeysBuffer.size() + BlockSize);
		
		std::copy_n((uint8_t *)(&KeySize), 2U, KeysBuffer.end() - BlockSize);
		std::copy_n((uint8_t *)(&kv.first[0]), KeySize, KeysBuffer.end() - BlockSize + 2U);
		std::copy_n((uint8_t *)(&ValueOffset), 8U, KeysBuffer.end() - 16U);
		std::copy_n((uint8_t *)(&ValueLength), 8U, KeysBuffer.end() - 8U);
	}
	if (! Handy::TryWriteFile(StorePath + ".Keys"s, KeysBuffer)) {
		Log.print("Warning: Failed to write Keys file.");
	}
}

//Re-build the values file, copying only the data referenced in the index (removes empty space in values file).
//Can only be called when the store is open. After densification, the store should be open (unless something failed)
inline bool SimpleKVStore::DensifyValuesFile() {
	if (! ValuesFile.IsOpen())
		return false;
	
	Handy::File NewValuesFile;
	if (! NewValuesFile.Open(StorePath + ".NewValues"s, true))
		return false;
	std::vector<std::string> KeysToRemove;
	for (auto & kv : Index) {
		std::string Key = kv.first;
		uint64_t OldValueOffset = std::get<0>(kv.second);
		uint64_t ValueLength    = std::get<1>(kv.second);
		
		std::vector<uint8_t> Value;
		if ((! ReadValue(OldValueOffset, ValueLength, Value)) || (Value.size() != ValueLength)) {
			Log.print("Dropping item during densification since value could not be read.");
			KeysToRemove.push_back(Key);
		}
		else {
			uint64_t NewValueOffset = (uint64_t) NewValuesFile.Tell(Handy::AccessPosition::Write);
			if (NewValuesFile.Write(Value))
				std::get<0>(kv.second) = NewValueOffset;
			else {
				Log.print("Dropping item during densification since value could not be written.");
				KeysToRemove.push_back(Key);
			}
		}
	}
	for (std::string Key : KeysToRemove)
		Index.erase(Key);
	NewValuesFile.Flush();
	NewValuesFile.Close();
	
	//Now, close the values file, replace it with NewValues, and re-open it
	ValuesFile.Flush();
	ValuesFile.Close();
	std::error_code ec;
	std::filesystem::rename(StorePath + ".Values"s, StorePath + ".OldValues"s, ec);
	if (! ec) {
		std::filesystem::rename(StorePath + ".NewValues"s, StorePath + ".Values"s, ec);
		if (! ec) {
			std::filesystem::remove(StorePath + ".OldValues"s, ec);
			if (ec)
				Log.print("Warning: During KV store densification, could not remove old values file.");
		}
		else {
			Log.print("Error: During KV store densification, could not replace old values file with new one. Trying to restore.");
			std::filesystem::rename(StorePath + ".OldValues"s, StorePath + ".Values"s, ec);
			if (! ec)
				Log.print("Restore was successful.");
			else
				Log.print("Restore failed.");
		}
	}
	else
		Log.print("Error: During KV store densification, could not rename values file. Store will not be modified.");
	if (! ValuesFile.Open(StorePath + ".Values"s, false)) {
		Log.print("Error: During KV store densification, could not open densified values file.");
		return false;
	}
	return true;
}

inline bool SimpleKVStore::ReadValue(uint64_t Offset, uint64_t Size, std::vector<uint8_t> & Value) {
	if (! ValuesFile.IsOpen())
		return false;
	ValuesFile.Seek(Handy::AccessPosition::Read, Offset);

	if (!ValuesFile.ReadTo(Value, Size))
	{
		/// FAILED
		Log.printf("ReadValue Failed. Tried reading bytes [%llu, %llu].",
		          (unsigned long long int) Offset,
		          (unsigned long long int) (Offset + Size - 1U));
		ValuesFile.SeekEnd(Handy::AccessPosition::Read);
		unsigned long long int maxIndex = (unsigned long long int) ValuesFile.Tell(Handy::AccessPosition::Read);
		Log.printf("Last index in Values file: %llu", maxIndex);

		return false;
	}

	return true;
}

inline bool SimpleKVStore::Get(std::string const & Key, std::vector<uint8_t> & Value) {
	std::scoped_lock slock(Mtx);
	if (Index.count(Key) == 0U)
		return false;
	bool success = ReadValue(std::get<0>(Index.at(Key)), std::get<1>(Index.at(Key)), Value);
	//If we couldn't read the value for this item, remove it from the index.
	if (! success)
		Index.erase(Key);
	return success;
}

inline bool SimpleKVStore::Put(std::string const & Key, std::vector<uint8_t> const & Value) {
	std::scoped_lock slock(Mtx);
	if (! ValuesFile.IsOpen())
		return false;
	ValuesFile.SeekEnd(Handy::AccessPosition::Write);
	uint64_t ValueOffset = (uint64_t) ValuesFile.Tell(Handy::AccessPosition::Write);
	if (ValuesFile.Write(Value)) {
		Index[Key] = std::make_tuple(ValueOffset, (uint64_t) Value.size());
		return true;
	}
	else {
		Log.print("Warning in SimpleKVStore::Put: Failed to write value to file.");
		return false;
	}
}

//Same as Put(), but checks to see if an item with the given key is already present and fails if it finds one (and returns false)
inline bool SimpleKVStore::PutIfNew(std::string const & Key, std::vector<uint8_t> const & Value) {
	std::scoped_lock slock(Mtx);
	if (Index.count(Key) > 0U)
		return false;
	if (! ValuesFile.IsOpen())
		return false;
	ValuesFile.SeekEnd(Handy::AccessPosition::Write);
	uint64_t ValueOffset = (uint64_t) ValuesFile.Tell(Handy::AccessPosition::Write);
	if (ValuesFile.Write(Value)) {
		Index[Key] = std::make_tuple(ValueOffset, (uint64_t) Value.size());
		return true;
	}
	else {
		Log.print("Warning in SimpleKVStore::Put: Failed to write value to file.");
		return false;
	}
}

inline void SimpleKVStore::RefreshSizesIfNeeded() {
	if (m_sizeRefreshStopwatch.SecondsF() > 2.0f) {
		if (ValuesFile.IsOpen()) {
			m_numBytes = 0U;
			for (auto const & kv : Index) {
				uint16_t KeySize = kv.first.size();
				uint64_t BlockSize = 18U + KeySize;
				uint64_t ValueLength = std::get<1>(kv.second);
				m_numBytes += BlockSize + ValueLength;
			}
			
			std::error_code ec1, ec2;
			uint64_t KeyFileSizeBytes   = (uint64_t) std::filesystem::file_size(StorePath + ".Keys"s, ec1);
			uint64_t ValueFileSizeBytes = (uint64_t) std::filesystem::file_size(StorePath + ".Values"s, ec2);
			if (ec1 || ec2)
				m_numBytesOnDisk = 0U;
			else
				m_numBytesOnDisk = KeyFileSizeBytes + ValueFileSizeBytes;
		}
		else {
			m_numBytes = 0U;
			m_numBytesOnDisk = 0U;
		}
		m_sizeRefreshStopwatch.Restart();
	}
}



