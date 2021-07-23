//Main Window and provider of outer draw loop - all other window draw loops are called
//from within our Draw member function.
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"
#include "soloud.h"

//Project Includes
#include "Drawable.hpp"
#include "Themes.hpp"
#include "../Journal.h"

class ReconUI : public AppUI, public DrawableHost {
	private:
		ReconUI();

	public:
		COPY_ASSIGN_MOVE_CTOR(ReconUI, delete, delete, delete)

		static ReconUI & Instance() { static ReconUI app; return app; }
		
		Journal * Log = nullptr;
		bool show_demo_window = false;
		
		std::mutex soLoudMutex; //Mutex for gSoloud object. Lock when accessing since library is not explicit about thread-safety
		SoLoud::Soloud gSoloud; //SoLoud engine
		
		virtual ~ReconUI();
		
		void Preframe()  override;
		void Draw()      override;
		void Postframe() override;
};



