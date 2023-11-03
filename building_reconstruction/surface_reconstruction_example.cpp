#include "Surface_reconstruction.h"
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point>;
using FT = Kernel::FT;

int main()
{
    Surface_reconstruction surface_reconstruction;
    surface_reconstruction.setInputFile("../data/Construction_home_sample_normals.ply");
//    Этот компонент обнаружения формы основан на алгоритме роста региона, применяемом к набору указанных пользователем элементов.
//            Фигуры определяются растущими областями из семян, где каждая область создается следующим образом:
//
//    -# Выберите следующий доступный товар;
//    -# Найти своих соседей в наборе данных;
//    -# Включите тех соседей, которые удовлетворяют требованиям региона;
//    -# Повторите процедуру для всех включенных соседей;
//    -# Если ни один из других соседей не удовлетворяет требованиям, начните новый регион.
    //100 Когда `sphere_radius` = 0,3 (c), получается лучший визуальный результат
    // (b) 17 регионов встречаются, когда `sphere_radius` = 0,1;
    //(c) 8 регионов встречаются, когда `sphere_radius` = 0,3;
    //(d) 4 региона найдены, когда `sphere_radius` = 1,2.
//    - `maximum_distance` - максимальное расстояние от точки до линии/плоскости;
//    - `maximum_angle` - максимальный угол в градусах между нормалью, связанной с точкой, и нормалью прямой/плоскости;
//    - `minimum_region_size` - минимальное количество точек, которое должен иметь регион.
    surface_reconstruction.setSimplificationAverageSpacing(0);
    surface_reconstruction.setEstimateNormalsNeighbors(0); //6
    surface_reconstruction.setSearchSphereRadius(FT(2) / FT(13)); // чем больше радиус, тем меньше регионов // 27
    surface_reconstruction.setMaxDistanceToPlane(FT(2) / FT(130));//1000 640 320 160 // 80 чем больше, тем дольше (меньше плоскостей)
    surface_reconstruction.setMaxAcceptedAngle(FT(25));
    surface_reconstruction.setMinRegionSize(30); // 40
    Surface_mesh surface_mesh = surface_reconstruction.reconstruct();
    return 0;
}