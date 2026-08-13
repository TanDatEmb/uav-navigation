#include "navigation_planning/a_star.hpp"

namespace navigation_planning {

SearchResult AStar::search(const navigation_mapping::WorldModel& model,
                           const SearchRequest& request) const {
  return detail::searchModel(model, request);
}

}  // namespace navigation_planning
