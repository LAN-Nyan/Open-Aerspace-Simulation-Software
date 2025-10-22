#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iomanip>

// ===================================================================
// 1. CORE DATA STRUCTURES
// ===================================================================

struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
};

struct Material {
    std::string material_id;
    double density;          // kg/m^3
    double yield_strength;   // Pa
    double youngs_modulus;   // Pa
    double poisson_ratio;
};

struct Boundary {
    std::string tag;
    std::string type;        // "INLET" or "OUTPUT"
    std::vector<int> mesh_faces;
    double pressure_pa;      // For INLET
    double flow_rate_kg_s;   // For OUTPUT
};

struct Environment {
    double altitude;         // meters
    double air_speed;        // m/s
    double temperature;      // Kelvin
};

struct SimConfig {
    std::string aero_mode;   // "Default" or "Advanced"
    Environment environment;
    std::vector<Boundary> boundaries;
};

struct MeshVertex {
    Vec3 position;
    Vec3 normal;
};

struct MeshFace {
    int face_id;
    std::vector<int> vertex_indices;
    std::string material_id;
    Vec3 center;
    Vec3 normal;
    double area;
};

struct Mesh3D {
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;
    std::string filename;
};

struct AeroResult {
    double lift_coefficient;
    double drag_coefficient;
    double lift_force;
    double drag_force;
    std::vector<double> pressure_per_face;
    double total_area;
};

struct StructuralResult {
    std::vector<double> stress_per_face;   // Pa
    std::vector<double> strain_per_face;
    double max_stress;
    double max_strain;
    double safety_factor;
    std::string critical_material;
};

// ===================================================================
// 2. 3D FILE LOADER (OBJ/STL)
// ===================================================================

class MeshLoader {
public:
    static Mesh3D loadOBJ(const std::string& filepath) {
        Mesh3D mesh;
        mesh.filename = filepath;
        
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open OBJ file: " + filepath);
        }
        
        std::string line;
        int face_count = 0;
        
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;
            
            if (prefix == "v") {
                // Vertex position
                MeshVertex vertex;
                iss >> vertex.position.x >> vertex.position.y >> vertex.position.z;
                mesh.vertices.push_back(vertex);
            }
            else if (prefix == "vn") {
                // Vertex normal (optional)
                Vec3 normal;
                iss >> normal.x >> normal.y >> normal.z;
                if (mesh.vertices.size() > 0) {
                    mesh.vertices.back().normal = normal;
                }
            }
            else if (prefix == "f") {
                // Face (triangle or quad)
                MeshFace face;
                face.face_id = face_count++;
                face.material_id = "default";
                
                std::string vertex_str;
                while (iss >> vertex_str) {
                    int v_idx = std::stoi(vertex_str.substr(0, vertex_str.find('/'))) - 1;
                    face.vertex_indices.push_back(v_idx);
                }
                
                // Calculate face center and normal
                if (face.vertex_indices.size() >= 3) {
                    Vec3 v0 = mesh.vertices[face.vertex_indices[0]].position;
                    Vec3 v1 = mesh.vertices[face.vertex_indices[1]].position;
                    Vec3 v2 = mesh.vertices[face.vertex_indices[2]].position;
                    
                    face.center.x = (v0.x + v1.x + v2.x) / 3.0;
                    face.center.y = (v0.y + v1.y + v2.y) / 3.0;
                    face.center.z = (v0.z + v1.z + v2.z) / 3.0;
                    
                    // Simple area calculation for triangle
                    Vec3 edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    Vec3 edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    Vec3 cross(
                        edge1.y * edge2.z - edge1.z * edge2.y,
                        edge1.z * edge2.x - edge1.x * edge2.z,
                        edge1.x * edge2.y - edge1.y * edge2.x
                    );
                    face.area = cross.magnitude() * 0.5;
                    face.normal = cross;
                    
                    mesh.faces.push_back(face);
                }
            }
        }
        
        file.close();
        std::cout << "Loaded OBJ: " << mesh.vertices.size() << " vertices, " 
                  << mesh.faces.size() << " faces\n";
        return mesh;
    }
    
    static Mesh3D loadSTL(const std::string& filepath) {
        Mesh3D mesh;
        mesh.filename = filepath;
        
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open STL file: " + filepath);
        }
        
        std::string line;
        int face_count = 0;
        
        while (std::getline(file, line)) {
            if (line.find("facet normal") != std::string::npos) {
                MeshFace face;
                face.face_id = face_count++;
                face.material_id = "default";
                
                // Parse normal
                std::istringstream iss(line);
                std::string dummy;
                iss >> dummy >> dummy >> face.normal.x >> face.normal.y >> face.normal.z;
                
                // Parse vertices
                std::getline(file, line); // outer loop
                for (int i = 0; i < 3; i++) {
                    std::getline(file, line);
                    std::istringstream vss(line);
                    std::string v_dummy;
                    Vec3 pos;
                    vss >> v_dummy >> pos.x >> pos.y >> pos.z;
                    
                    MeshVertex vertex;
                    vertex.position = pos;
                    vertex.normal = face.normal;
                    mesh.vertices.push_back(vertex);
                    face.vertex_indices.push_back(mesh.vertices.size() - 1);
                }
                
                // Calculate face center and area
                Vec3 v0 = mesh.vertices[face.vertex_indices[0]].position;
                Vec3 v1 = mesh.vertices[face.vertex_indices[1]].position;
                Vec3 v2 = mesh.vertices[face.vertex_indices[2]].position;
                
                face.center.x = (v0.x + v1.x + v2.x) / 3.0;
                face.center.y = (v0.y + v1.y + v2.y) / 3.0;
                face.center.z = (v0.z + v1.z + v2.z) / 3.0;
                
                Vec3 edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                Vec3 edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                Vec3 cross(
                    edge1.y * edge2.z - edge1.z * edge2.y,
                    edge1.z * edge2.x - edge1.x * edge2.z,
                    edge1.x * edge2.y - edge1.y * edge2.x
                );
                face.area = cross.magnitude() * 0.5;
                
                mesh.faces.push_back(face);
            }
        }
        
        file.close();
        std::cout << "Loaded STL: " << mesh.vertices.size() << " vertices, " 
                  << mesh.faces.size() << " faces\n";
        return mesh;
    }
};

// ===================================================================
// 3. BRUSH TOOL - Material Painting
// ===================================================================

class BrushTool {
private:
    Mesh3D* mesh_ptr;
    
public:
    void setMesh(Mesh3D* mesh) {
        mesh_ptr = mesh;
    }
    
    // Paint material by face indices
    void paintByFaces(const std::vector<int>& face_ids, const std::string& material_id) {
        if (!mesh_ptr) return;
        
        int painted = 0;
        for (int face_id : face_ids) {
            if (face_id >= 0 && face_id < mesh_ptr->faces.size()) {
                mesh_ptr->faces[face_id].material_id = material_id;
                painted++;
            }
        }
        std::cout << "Brush Tool: Painted " << painted << " faces with " 
                  << material_id << "\n";
    }
    
    // Paint material by region (e.g., all faces within bounds)
    void paintByRegion(Vec3 min_bound, Vec3 max_bound, const std::string& material_id) {
        if (!mesh_ptr) return;
        
        int painted = 0;
        for (auto& face : mesh_ptr->faces) {
            if (face.center.x >= min_bound.x && face.center.x <= max_bound.x &&
                face.center.y >= min_bound.y && face.center.y <= max_bound.y &&
                face.center.z >= min_bound.z && face.center.z <= max_bound.z) {
                face.material_id = material_id;
                painted++;
            }
        }
        std::cout << "Brush Tool: Painted " << painted << " faces in region with " 
                  << material_id << "\n";
    }
    
    // Paint entire mesh with material
    void paintAll(const std::string& material_id) {
        if (!mesh_ptr) return;
        
        for (auto& face : mesh_ptr->faces) {
            face.material_id = material_id;
        }
        std::cout << "Brush Tool: Painted all " << mesh_ptr->faces.size() 
                  << " faces with " << material_id << "\n";
    }
};

// ===================================================================
// 4. INLET/OUTPUT MANAGER
// ===================================================================

class BoundaryManager {
private:
    std::vector<Boundary> boundaries;
    
public:
    void addInlet(const std::string& tag, const std::vector<int>& face_ids, 
                  double pressure_pa) {
        Boundary inlet;
        inlet.tag = tag;
        inlet.type = "INLET";
        inlet.mesh_faces = face_ids;
        inlet.pressure_pa = pressure_pa;
        boundaries.push_back(inlet);
        
        std::cout << "Added INLET '" << tag << "' with " << face_ids.size() 
                  << " faces at " << pressure_pa << " Pa\n";
    }
    
    void addOutput(const std::string& tag, const std::vector<int>& face_ids,
                   double flow_rate_kg_s) {
        Boundary output;
        output.tag = tag;
        output.type = "OUTPUT";
        output.mesh_faces = face_ids;
        output.flow_rate_kg_s = flow_rate_kg_s;
        boundaries.push_back(output);
        
        std::cout << "Added OUTPUT '" << tag << "' with " << face_ids.size() 
                  << " faces at " << flow_rate_kg_s << " kg/s\n";
    }
    
    const std::vector<Boundary>& getBoundaries() const {
        return boundaries;
    }
    
    void clearAll() {
        boundaries.clear();
        std::cout << "Cleared all inlet/output definitions\n";
    }
};

// ===================================================================
// 5. SIMPLE JSON PARSER
// ===================================================================

class SimpleConfigParser {
private:
    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\n\r\"");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\n\r\",");
        return str.substr(first, last - first + 1);
    }
    
    double parseDouble(const std::string& str) {
        return std::stod(trim(str));
    }
    
public:
    std::map<std::string, Material> parseMaterials(const std::string& filepath) {
        std::map<std::string, Material> materials;
        std::ifstream file(filepath);
        
        if (!file.is_open()) {
            std::cout << "Using default materials (file not found)\n";
            return getDefaultMaterials();
        }
        
        std::string content, line;
        while (std::getline(file, line)) content += line;
        file.close();
        
        size_t pos = 0;
        while ((pos = content.find("\"material_id\":", pos)) != std::string::npos) {
            Material mat;
            
            size_t id_start = content.find("\"", pos + 14) + 1;
            size_t id_end = content.find("\"", id_start);
            mat.material_id = content.substr(id_start, id_end - id_start);
            
            size_t dens_pos = content.find("\"density\":", id_end);
            size_t dens_start = content.find(":", dens_pos) + 1;
            size_t dens_end = content.find_first_of(",\n}", dens_start);
            mat.density = parseDouble(content.substr(dens_start, dens_end - dens_start));
            
            size_t yield_pos = content.find("\"yield_strength\":", dens_end);
            size_t yield_start = content.find(":", yield_pos) + 1;
            size_t yield_end = content.find_first_of(",\n}", yield_start);
            mat.yield_strength = parseDouble(content.substr(yield_start, yield_end - yield_start));
            
            size_t young_pos = content.find("\"youngs_modulus\":", yield_end);
            size_t young_start = content.find(":", young_pos) + 1;
            size_t young_end = content.find_first_of(",\n}", young_start);
            mat.youngs_modulus = parseDouble(content.substr(young_start, young_end - young_start));
            
            size_t poisson_pos = content.find("\"poisson_ratio\":", young_end);
            size_t poisson_start = content.find(":", poisson_pos) + 1;
            size_t poisson_end = content.find_first_of(",\n}", poisson_start);
            mat.poisson_ratio = parseDouble(content.substr(poisson_start, poisson_end - poisson_start));
            
            materials[mat.material_id] = mat;
            pos = poisson_end;
        }
        
        return materials.empty() ? getDefaultMaterials() : materials;
    }
    
    SimConfig parseSimConfig(const std::string& filepath) {
        SimConfig config;
        config.aero_mode = "Default";
        config.environment = {0.0, 100.0, 288.15};
        
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cout << "Using default simulation config\n";
            return config;
        }
        
        std::string content, line;
        while (std::getline(file, line)) content += line;
        file.close();
        
        size_t mode_pos = content.find("\"aero_mode\":");
        if (mode_pos != std::string::npos) {
            size_t mode_start = content.find("\"", mode_pos + 12) + 1;
            size_t mode_end = content.find("\"", mode_start);
            config.aero_mode = content.substr(mode_start, mode_end - mode_start);
        }
        
        return config;
    }
    
    std::map<std::string, Material> getDefaultMaterials() {
        std::map<std::string, Material> defaults;
        defaults["Composite_CFRP"] = {"Composite_CFRP", 1550.0, 600.0e6, 70.0e9, 0.3};
        defaults["Aluminum_7075"] = {"Aluminum_7075", 2810.0, 503.0e6, 71.7e9, 0.33};
        defaults["Steel"] = {"Steel", 7850.0, 250.0e6, 200.0e9, 0.3};
        return defaults;
    }
};

// ===================================================================
// 6. AERODYNAMICS SOLVER
// ===================================================================

class AeroSolver {
protected:
    SimConfig config;
    
    double calculateDynamicPressure() {
        double rho = 1.225; // Sea level
        return 0.5 * rho * config.environment.air_speed * config.environment.air_speed;
    }
    
public:
    virtual ~AeroSolver() = default;
    virtual void initialize(const SimConfig& cfg) { config = cfg; }
    virtual AeroResult solve(const Mesh3D& mesh, double angle_of_attack) = 0;
};

class DefaultAeroSolver : public AeroSolver {
private:
    std::map<double, std::pair<double, double>> lookup_table;
    
    void buildLookupTable() {
        lookup_table[-5.0] = {-0.55, 0.010};
        lookup_table[0.0] = {0.0, 0.008};
        lookup_table[5.0] = {0.55, 0.010};
        lookup_table[10.0] = {1.0, 0.015};
        lookup_table[15.0] = {1.3, 0.025};
    }
    
public:
    void initialize(const SimConfig& cfg) override {
        AeroSolver::initialize(cfg);
        buildLookupTable();
    }
    
    AeroResult solve(const Mesh3D& mesh, double aoa) override {
        AeroResult result;
        
        // Interpolate coefficients
        double cl = 0.1 * aoa;  // Simplified
        double cd = 0.008 + 0.001 * aoa * aoa;
        
        result.lift_coefficient = cl;
        result.drag_coefficient = cd;
        
        // Calculate total area
        result.total_area = 0.0;
        for (const auto& face : mesh.faces) {
            result.total_area += face.area;
        }
        
        double q = calculateDynamicPressure();
        result.lift_force = cl * q * result.total_area;
        result.drag_force = cd * q * result.total_area;
        
        // Calculate pressure per face
        for (const auto& face : mesh.faces) {
            double local_pressure = q * cl * (face.area / result.total_area);
            result.pressure_per_face.push_back(local_pressure);
        }
        
        return result;
    }
};

// ===================================================================
// 7. STRUCTURAL SOLVER
// ===================================================================

class StructuralSolver {
private:
    std::map<std::string, Material> material_db;
    
public:
    void initialize(const std::map<std::string, Material>& materials) {
        material_db = materials;
    }
    
    StructuralResult solve(const Mesh3D& mesh, const AeroResult& aero_loads) {
        StructuralResult result;
        result.max_stress = 0.0;
        result.max_strain = 0.0;
        
        for (size_t i = 0; i < mesh.faces.size(); i++) {
            const auto& face = mesh.faces[i];
            
            if (material_db.count(face.material_id) == 0) continue;
            const Material& mat = material_db.at(face.material_id);
            
            // Apply aerodynamic pressure to face
            double pressure = (i < aero_loads.pressure_per_face.size()) 
                             ? aero_loads.pressure_per_face[i] : 0.0;
            double stress = pressure; // Simplified: stress = pressure
            double strain = stress / mat.youngs_modulus;
            
            result.stress_per_face.push_back(stress);
            result.strain_per_face.push_back(strain);
            
            if (stress > result.max_stress) {
                result.max_stress = stress;
                result.critical_material = mat.material_id;
            }
            if (strain > result.max_strain) {
                result.max_strain = strain;
            }
        }
        
        // Calculate safety factor
        if (!mesh.faces.empty() && material_db.count(mesh.faces[0].material_id)) {
            double yield = material_db.at(result.critical_material).yield_strength;
            result.safety_factor = yield / result.max_stress;
        }
        
        return result;
    }
};

// ===================================================================
// 8. MAIN SIMULATION ENGINE
// ===================================================================

class SimulationEngine {
private:
    SimpleConfigParser parser;
    std::unique_ptr<AeroSolver> aero_solver;
    StructuralSolver struct_solver;
    BrushTool brush_tool;
    BoundaryManager boundary_mgr;
    
    std::map<std::string, Material> material_database;
    SimConfig current_config;
    Mesh3D mesh;
    
public:
    SimulationEngine() {
        aero_solver = std::make_unique<DefaultAeroSolver>();
    }
    
    void loadConfiguration(const std::string& materials_path,
                          const std::string& sim_config_path) {
        std::cout << "\n=== Loading Configuration ===\n";
        material_database = parser.parseMaterials(materials_path);
        current_config = parser.parseSimConfig(sim_config_path);
        
        std::cout << "Materials loaded: " << material_database.size() << "\n";
        std::cout << "Aero mode: " << current_config.aero_mode << "\n";
        
        aero_solver->initialize(current_config);
        struct_solver.initialize(material_database);
    }
    
    void loadMesh(const std::string& filepath) {
        std::cout << "\n=== Loading 3D Model ===\n";
        
        if (filepath.substr(filepath.length() - 4) == ".obj") {
            mesh = MeshLoader::loadOBJ(filepath);
        } else if (filepath.substr(filepath.length() - 4) == ".stl") {
            mesh = MeshLoader::loadSTL(filepath);
        } else {
            throw std::runtime_error("Unsupported file format. Use .obj or .stl");
        }
        
        brush_tool.setMesh(&mesh);
    }
    
    BrushTool& getBrushTool() { return brush_tool; }
    BoundaryManager& getBoundaryManager() { return boundary_mgr; }
    
    AeroResult runAeroSimulation(double angle_of_attack) {
        std::cout << "\n=== Aerodynamic Simulation ===\n";
        std::cout << "Mode: " << current_config.aero_mode << "\n";
        std::cout << "Angle of Attack: " << angle_of_attack << "°\n";
        
        AeroResult result = aero_solver->solve(mesh, angle_of_attack);
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "CL: " << result.lift_coefficient << "\n";
        std::cout << "CD: " << result.drag_coefficient << "\n";
        std::cout << "Lift: " << result.lift_force << " N\n";
        std::cout << "Drag: " << result.drag_force << " N\n";
        
        return result;
    }
    
    StructuralResult runStructuralAnalysis(const AeroResult& aero_loads) {
        std::cout << "\n=== Structural Analysis ===\n";
        
        StructuralResult result = struct_solver.solve(mesh, aero_loads);
        
        std::cout << "Max Stress: " << result.max_stress / 1e6 << " MPa\n";
        std::cout << "Critical Material: " << result.critical_material << "\n";
        std::cout << "Safety Factor: " << result.safety_factor << "\n";
        
        return result;
    }
    
    void exportResults(const std::string& filename, const AeroResult& aero,
                      const StructuralResult& structure) {
        std::ofstream out(filename);
        out << "Open Aerspace Sim - Results\n";
        out << "============================\n\n";
        out << "Aerodynamics:\n";
        out << "  CL: " << aero.lift_coefficient << "\n";
        out << "  CD: " << aero.drag_coefficient << "\n";
        out << "  Lift: " << aero.lift_force << " N\n";
        out << "  Drag: " << aero.drag_force << " N\n\n";
        out << "Structural:\n";
        out << "  Max Stress: " << structure.max_stress / 1e6 << " MPa\n";
        out << "  Safety Factor: " << structure.safety_factor << "\n";
        out.close();
        
        std::cout << "\nResults exported to: " << filename << "\n";
    }
};

// ===================================================================
// 9. MAIN PROGRAM - DEMO
// ===================================================================

int main() {
    std::cout << "============================================\n";
    std::cout << "  Open Aerspace Sim - BETA 0.01\n";
    std::cout << "  Beginner-Friendly Aerodynamic Simulation\n";
    std::cout << "  Release: 10/31/25\n";
    std::cout << "============================================\n";
    
    try {
        SimulationEngine engine;
        
        // Load configuration
        engine.loadConfiguration("materials.json", "sim_config.json");
        
        // DEMO: Create a simple mesh if no file provided
        std::cout << "\n[DEMO MODE - Using test geometry]\n";
        std::cout << "In production: engine.loadMesh(\"your_model.obj\");\n";
        
        // Simulate loading a simple wing mesh
        Mesh3D demo_mesh;
        for (int i = 0; i < 50; i++) {
            MeshFace face;
            face.face_id = i;
            face.material_id = "default";
            face.center = Vec3(i * 0.1, 0, 0);
            face.area = 0.05;
            demo_mesh.faces.push_back(face);
        }
        std::cout << "Demo mesh: 50 faces created\n";
        
        // Use Brush Tool to paint materials
        std::cout << "\n=== Using Brush Tool ===\n";
        BrushTool& brush = engine.getBrushTool();
        brush.setMesh(&demo_mesh);
        brush.paintByFaces({0, 1, 2, 3, 4}, "Composite_CFRP");
        brush.paintByFaces({5, 6, 7, 8, 9}, "Aluminum_7075");
        brush.paintByRegion(Vec3(1.0, -1, -1), Vec3(3.0, 1, 1), "Steel");
        
        // Define Inlet/Output
        std::cout << "\n=== Defining Boundaries ===\n";
        BoundaryManager& boundaries = engine.getBoundaryManager();
        boundaries.addInlet("Main_Intake", {10, 11, 12}, 101325.0);
        boundaries.addOutput("Rear_Exhaust", {40, 41, 42}, 5.0);
        
        // Run simulations at different angles
        for (double aoa : {0.0, 5.0, 10.0}) {
            AeroResult aero = engine.runAeroSimulation(aoa);
            StructuralResult structure = engine.runStructuralAnalysis(aero);
            
            engine.exportResults("results_aoa_" + std::to_string((int)aoa) + ".txt",
                               aero, structure);
        }
        
        std::cout << "\n============================================\n";
        std::cout << "  Simulation Complete!\n";
        std::cout << "  Check results_*.txt for detailed output\n";
        std::cout << "============================================\n";
        
    } catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
