//Memory cache for satellite tiles - keeps track of the last time an item was accessed and allows you to purge old items

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "CacheMem.hpp"
#include "../UI/TextureUploadFlowRestrictor.hpp"

namespace Maps {
	//Without C++17, static constexpr members of structs and classes have external linkage, and so must be defined in some translation unit.
	//We appear to kind of get away without this if we compile with high optimization levels since all references to the symbol get
	//optimized out, but technically we need this.
	constexpr std::chrono::seconds CacheMem::ExpirationTimeSeconds;

	void CacheMem::Add(Tile tile, SatelliteSource source, std::vector<uint8_t> & data, int width, int height) {
		TextureUploadFlowRestrictor::Instance().WaitUntilUploadIsAllowed();
		ImTextureID i2 = ImGuiApp::Instance().CreateImageRGBA8888(&data[0], width, height);
		
		std::lock_guard<std::mutex> lock(m_mutex);
		m_tiles[std::make_tuple(tile,source)] = std::tuple<ImTextureID, TimePoint>(i2, std::chrono::system_clock::now());
	}

	void CacheMem::Add(Tile tile, SatelliteSource source, uint8_t * data, int width, int height) {
		TextureUploadFlowRestrictor::Instance().WaitUntilUploadIsAllowed();
		ImTextureID i2 = ImGuiApp::Instance().CreateImageRGBA8888(&data[0], width, height);
		
		std::lock_guard<std::mutex> lock(m_mutex);
		m_tiles[std::make_tuple(tile,source)] = std::tuple<ImTextureID, TimePoint>(i2, std::chrono::system_clock::now());
	}

	bool CacheMem::Has(Tile tile, SatelliteSource source) {
		std::lock_guard<std::mutex> lock(m_mutex);

		return Handy::Contains(m_tiles, std::make_tuple(tile,source));
	}

	ImTextureID CacheMem::Get(Tile tile, SatelliteSource source) {
		std::lock_guard<std::mutex> lock(m_mutex);

		auto res = Handy::TryGet(m_tiles, std::make_tuple(tile,source));
		if (res) {
			//std::tuple<ImTextureID, TimePoint> imageDT = *res;
			std::get<1>(*res) = std::chrono::system_clock::now();
			m_tiles[std::make_tuple(tile,source)] = *res;

			return std::get<0>(*res);
		}

		return nullptr;
	}

	void CacheMem::PurgeAll() {
		std::vector<ImTextureID> toRemoveImg;
		
		{
			std::scoped_lock lock(m_mutex);
			for (auto & item : m_tiles)
				toRemoveImg.push_back(std::get<0>(item.second));
			m_tiles.clear();
		}
		
		ImGuiApp::Instance().DeleteImagesAsync(toRemoveImg);
	}
	
	//Discard expired tiles (up to the given fraction of the current cache size). Will make sure to throw out at least some expired tiles
	//if there are any, even if this is 0.0
	void CacheMem::PurgeExpired(double MaxFractionToPurge) {
		TimePoint now = std::chrono::system_clock::now();
		
		std::vector<ImTextureID> texturesToRemove;
		{
			std::scoped_lock lock(m_mutex);
			size_t maxItemsToRemove = std::max(size_t(MaxFractionToPurge * double(m_tiles.size())), size_t(5));
			
			std::vector<std::tuple<Tile, SatelliteSource>> keysToRemove;
			for (auto const & item : m_tiles) {
				Tile        tile = std::get<0>(item.first);
				ImTextureID img  = std::get<0>(item.second);
				TimePoint   tp   = std::get<1>(item.second);

				auto ageSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - tp);
				if (ageSeconds > ExpirationTimeSeconds && tile.Zoom > 2) {
					keysToRemove.push_back(item.first);
					texturesToRemove.push_back(img);
					if (keysToRemove.size() >= maxItemsToRemove)
						break;
				}
			}
			
			for (auto key : keysToRemove)
				m_tiles.erase(key);
		}
		ImGuiApp::Instance().DeleteImagesAsync(texturesToRemove);
	}

}


