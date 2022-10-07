//Tool for regulating threads uploading textures off the draw thread - avoids upload surges
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <mutex>
#include <atomic>
#include <condition_variable>

//Warning: Do not call WaitUntilUploadIsAllowed() from the main thread or you may deadlock.
//Also, ensure that any thread that does call WaitUntilUploadIsAllowed() isn't holding any locks that
//might hold up the main thread or you may also deadlock. Note: We might want to re-architect this to
//have WaitUntilUploadIsAllowed() timeout after some small amount of time if it isn't reset. This would
//be a failsafe against deadlocks from unexpected thread interactions like those just mentioned. For now
//though, lets keep it the way it is so we can track down those bugs and fix them rather than hide them.
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

