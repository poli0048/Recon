//Tool for regulating threads uploading textures off the draw thread - avoids upload surges
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <mutex>
#include <atomic>
#include <condition_variable>

//Warning: Do not call WaitUntilUploadIsAllowed() from the main thread or you may deadlock.
class TextureUploadFlowRestrictor {
	private:
		static constexpr unsigned int MaxUploadsPerFrame = 25U;
		
		std::mutex m_mtx;
		std::condition_variable m_cv;
		unsigned int m_uploadCount = 0U;
		bool m_abort = false;
		
	public:
		static TextureUploadFlowRestrictor & Instance() { static TextureUploadFlowRestrictor Obj; return Obj; }
		
		TextureUploadFlowRestrictor() = default;
		~TextureUploadFlowRestrictor() {
			std::unique_lock<std::mutex> lck(m_mtx);
			m_abort = true;
			m_cv.notify_all();
		}
		
		void WaitUntilUploadIsAllowed() {
			std::unique_lock<std::mutex> lck(m_mtx);
  			while ((m_uploadCount > MaxUploadsPerFrame) && (!m_abort))
  				m_cv.wait(lck);
  			m_uploadCount++;
		}
		
		//Reset should be called each frame (ideally after the main draw pass) - this allows some more uploads to take place
		void Reset() {
			std::unique_lock<std::mutex> lck(m_mtx);
			m_uploadCount = 0U;
			m_cv.notify_all();
		}
};

