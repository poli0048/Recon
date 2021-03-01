#pragma once

//System Includes
#include <vector>
#include <unordered_set>

//External Includes
#include "../../../handycpp/Handy.hpp"

class Drawable {
	public:
		std::string DrawableName; //Typically the name/ID of the window if drawable is a window
		bool IsAlive = true; //Setting to false will trigger destruction by host
		bool IsVisible = true;
		
		virtual void Draw() = 0;
		virtual ~Drawable() = default;
};

class DrawableHost {
	private:
		std::vector<Drawable *> m_drawables;
		std::unordered_set<std::string> m_drawableNames;

	public:
		void DrawChildren() {
			std::vector<Drawable *> toRemove;

			for (Drawable * win : m_drawables) {
				if (!win->IsAlive)
					toRemove.push_back(win);
				else if (win->IsVisible)
					win->Draw();
			}

			for (Drawable * win : toRemove) {
				m_drawableNames.erase(win->DrawableName);
				Handy::RemoveFirstOf(m_drawables, win);
				Handy::SafeDelete(win);
			}
		}

		void Add(Drawable * d) { m_drawables.push_back(d); m_drawableNames.insert(d->DrawableName);}
		bool HasDrawable(std::string const & DrawableName) { return (m_drawableNames.count(DrawableName) > 0U); }

		virtual ~DrawableHost() {
			for (Drawable * win : m_drawables)
				Handy::SafeDelete(win);
		}
};


