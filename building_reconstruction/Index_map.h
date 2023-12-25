

#ifndef URBANRECONSTRUCTION_INDEX_MAP_H
#define URBANRECONSTRUCTION_INDEX_MAP_H

class Index_map {

public:
    using key_type = std::size_t;
    using value_type = int;
    using reference = value_type;
    using category = boost::readable_property_map_tag;

    Index_map() {}

    template<typename PointRange>
    Index_map(const PointRange &points,
              const std::vector <std::vector<std::size_t>> &regions)
            : m_indices(new std::vector<int>(points.size(), -1)) {
        for (std::size_t i = 0; i < regions.size(); ++i)
            for (const std::size_t idx: regions[i])
                (*m_indices)[idx] = static_cast<int>(i);
    }

    inline friend value_type get(const Index_map &index_map,
                                 const key_type key) {
        const auto &indices = *(index_map.m_indices);
        return indices[key];
    }

private:
    std::shared_ptr <std::vector<int>> m_indices;
};


#endif //URBANRECONSTRUCTION_INDEX_MAP_H
