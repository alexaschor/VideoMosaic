#ifndef FIELD_H
#define FIELD_H

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <queue>

#include "SETTINGS.h"

using namespace std;

class AABB: public AlignedBox<Real, 3> {
public:
    using AlignedBox<Real, 3>::AlignedBox;
    AABB(VEC3F c1, VEC3F c2): AlignedBox<Real, 3>(c1.cwiseMin(c2), c1.cwiseMax(c2)) {}
    AABB(): AABB(VEC3F(0,0,0), VEC3F(0,0,0)){}

    static AABB insideOut() {
        Real realMax = numeric_limits<Real>::max();
        Real realMin = numeric_limits<Real>::min();

        AABB out{};
        out.max() = VEC3F(realMin, realMin, realMin);
        out.min() = VEC3F(realMax, realMax, realMax);

        return out;
    }

    static VEC3F transferPoint(VEC3F pt, AABB from, AABB to) {
        if (!from.contains(pt)) {
            PRINT("AABB:transferPoint called, but the point is not contained by the origin box!");
            exit(1);
        }

        VEC3F proportional = (pt - from.min()).cwiseQuotient(from.span());

        return to.min() + proportional.cwiseProduct(to.span());
    }

    VEC3F span() const {
        return max()-min();
    }

    VEC3F clamp(VEC3F pos) const {
        return pos.cwiseMax(min()).cwiseMin(max());
    }

    void include(VEC3F p) {
        max() = max().cwiseMax(p);
        min() = min().cwiseMin(p);
    }

    void setCenter(VEC3F newCenter) {
        VEC3F offset = newCenter - this->center();
        max() += offset;
        min() += offset;
    }

    VEC3F randomPointInside() const {
        VEC3F pos;
        pos.x() = ((Real)rand() / RAND_MAX) * (max().x() - min().x()) + min().x();
        pos.y() = ((Real)rand() / RAND_MAX) * (max().y() - min().y()) + min().y();
        pos.z() = ((Real)rand() / RAND_MAX) * (max().z() - min().z()) + min().z();
        return pos;
    }

    vector<AABB> subdivideOctree() {
        vector<AABB> octree;

        //     CEIL    E---F
        //           / |4|5|
        //          /  +-+-+
        //         /   |7|6|
        //        /    H---G
        // FLOOR A---B    /
        //       |0|1|   /   point I is right smack dab in the middle
        //       +-+-+  /
        //       |3|2| /
        //       D---C
        //

        VEC3F A = this->corner(TopLeftFloor);
        VEC3F B = this->corner(TopRightFloor);
        VEC3F C = this->corner(BottomRightFloor);
        VEC3F D = this->corner(BottomLeftFloor);

        VEC3F E = this->corner(TopLeftCeil);
        VEC3F F = this->corner(TopRightCeil);
        VEC3F G = this->corner(BottomRightCeil);
        VEC3F H = this->corner(BottomLeftCeil);

        VEC3F I = this->center();

        octree.push_back(AABB(A, I)); // 0
        octree.push_back(AABB(B, I)); // 1
        octree.push_back(AABB(C, I)); // 2
        octree.push_back(AABB(D, I)); // 3

        octree.push_back(AABB(E, I)); // 4
        octree.push_back(AABB(F, I)); // 5
        octree.push_back(AABB(G, I)); // 6
        octree.push_back(AABB(H, I)); // 7

        return octree;

    }

};

class FieldFunction3D {
private:
    Real (*fieldFunction)(VEC3F pos);
public:
    FieldFunction3D(Real (*fieldFunction)(VEC3F pos)):fieldFunction(fieldFunction) {}

    FieldFunction3D():fieldFunction(nullptr) {} // Only to be used by subclasses

    virtual ~FieldFunction3D(){}

    virtual Real getFieldValue(const VEC3F& pos) const {
        return fieldFunction(pos);
    }

    virtual Real operator()(const VEC3F& pos) const {
        return getFieldValue(pos);
    }

    virtual VEC3F getNumericalGradient(const VEC3F& pos, Real eps) const {
        Real x = pos[0];
        Real y = pos[1];
        Real z = pos[2];

        Real xGrad = (getFieldValue(VEC3F(x - eps, y, z)) - getFieldValue(VEC3F(x + eps, y, z))) / (2*eps);
        Real yGrad = (getFieldValue(VEC3F(x, y - eps, z)) - getFieldValue(VEC3F(x, y + eps, z))) / (2*eps);
        Real zGrad = (getFieldValue(VEC3F(x, y, z - eps)) - getFieldValue(VEC3F(x, y, z + eps))) / (2*eps);

        return VEC3F(xGrad, yGrad, zGrad);
    }
};

class VectorField3D {
private:
    VEC3F (*vecFieldFunction)(VEC3F pos);

    class VecFieldSubField: public FieldFunction3D {
    private:
        VectorField3D* vecField;
        unsigned index;
    public:
        VecFieldSubField(VectorField3D* vecField, unsigned index): vecField(vecField), index(index) {}
        virtual Real getFieldValue(const VEC3F& pos) const { return vecField->getFieldValue(pos)[index]; }
    };

    class VecFieldMagField: public FieldFunction3D {
    private:
        VectorField3D* vecField;
    public:
        VecFieldMagField(VectorField3D* vecField): vecField(vecField) {}
        virtual Real getFieldValue(const VEC3F& pos) const { return vecField->getFieldValue(pos).norm(); }
    };

public:
    VectorField3D(VEC3F (*vecFieldFunction)(VEC3F pos)):vecFieldFunction(vecFieldFunction) {
        this->x = new VecFieldSubField(this, 0);
        this->y = new VecFieldSubField(this, 1);
        this->z = new VecFieldSubField(this, 2);

        this->mag = new VecFieldMagField(this);
    }

    VectorField3D():vecFieldFunction(nullptr) {
        this->x = new VecFieldSubField(this, 0);
        this->y = new VecFieldSubField(this, 1);
        this->z = new VecFieldSubField(this, 2);

        this->mag = new VecFieldMagField(this);
    } // Only to be used by subclasses


    virtual FieldFunction3D* operator[](size_t idx) {
        switch (idx) {
        case 0:
            return this->x;
            break;
        case 1:
            return this->y;
            break;
        case 2:
            return this->z;
            break;
        default:
            PRINT("Error: bad index to VecField operator[]");
            exit(1);
        }
    }

    virtual ~VectorField3D(){
        // delete x;
        // delete y;
        // delete z;
        // delete mag;
    }

    virtual VEC3F getFieldValue(const VEC3F& pos) const {
        return vecFieldFunction(pos);
    }

    virtual VEC3F operator()(const VEC3F& pos) const {
        return getFieldValue(pos);
    }

    FieldFunction3D *x, *y, *z, *mag;

    virtual void writeCSVPairs(string filename, uint xRes, uint yRes, uint zRes, VEC3F fieldMin, VEC3F fieldMax) {
        ofstream out;
        out.open(filename);
        if (out.is_open() == false)
            return;

        for (uint j = 0; j < xRes; ++j) {
            for (uint k = 0; k < yRes; ++k) {
                for (uint l = 0; l < zRes; ++l) {
                    VEC3F sampleOffset = fieldMax - fieldMin;
                    sampleOffset.x() *= ((float) j / xRes);
                    sampleOffset.y() *= ((float) k / yRes);
                    sampleOffset.z() *= ((float) l / zRes);

                    VEC3F samplePoint = fieldMin + sampleOffset;
                    VEC3F value = getFieldValue(samplePoint);

                    out <<
                        samplePoint.x() << "," <<
                        samplePoint.y() << "," <<
                        samplePoint.z() << "," <<
                        value.x()       << "," <<
                        value.y()       << "," <<
                        value.z()       << "," <<
                        endl;


                }
            }
        }

        printf("Wrote %d x %d x %d field (%d values) to %s\n", xRes, yRes, zRes, (xRes * yRes * zRes), filename.c_str());

        out.close();

    }

};

class MatrixField3D {
private:
    MAT3F (*matFieldFunction)(VEC3F pos);

    class MatFieldSubField: public VectorField3D {
    private:
        MatrixField3D* matField;
        unsigned index;
    public:
        MatFieldSubField(MatrixField3D* matField, unsigned index): matField(matField), index(index) {}
        virtual VEC3F getFieldValue(const VEC3F& pos) const { return matField->getFieldValue(pos).row(index); }
    };

    class MatFieldSpectralNormField: public FieldFunction3D {
    private:
        MatrixField3D* matField;
    public:
        MatFieldSpectralNormField(MatrixField3D* matField): matField(matField) {}
        virtual Real getFieldValue(const VEC3F& pos) const {
            MAT3F m = matField->getFieldValue(pos);
            return sqrtf((m.transpose() * m).eigenvalues().cwiseAbs().maxCoeff());
        }
    };

public:
    MatrixField3D(MAT3F (*matFieldFunction)(VEC3F pos)):matFieldFunction(matFieldFunction) {
        this->x = new MatFieldSubField(this, 0);
        this->y = new MatFieldSubField(this, 1);
        this->z = new MatFieldSubField(this, 2);

        this->spectralNorm = new MatFieldSpectralNormField(this);
    }

    MatrixField3D():matFieldFunction(nullptr) {
        this->x = new MatFieldSubField(this, 0);
        this->y = new MatFieldSubField(this, 1);
        this->z = new MatFieldSubField(this, 2);

        this->spectralNorm = new MatFieldSpectralNormField(this);
    } // Only to be used by subclasses


    virtual VectorField3D* operator[](size_t idx) {
        switch (idx) {
        case 0:
            return this->x;
            break;
        case 1:
            return this->y;
            break;
        case 2:
            return this->z;
            break;
        default:
            PRINT("Error: bad index to MatField operator[]");
            exit(1);
        }
    }

    virtual ~MatrixField3D(){
        // delete x;
        // delete y;
        // delete z;
        // delete spectralNorm;
    }

    virtual MAT3F getFieldValue(const VEC3F& pos) const {
        return matFieldFunction(pos);
    }

    virtual MAT3F operator()(const VEC3F& pos) const {
        return getFieldValue(pos);
    }

    VectorField3D *x, *y, *z;
    FieldFunction3D *spectralNorm;

};


class IteratedVF3D: public VectorField3D {
public:
    VectorField3D* field;
    unsigned iterations;
    IteratedVF3D(VectorField3D* field, unsigned iterations): field(field), iterations(iterations) {}

    virtual VEC3F getFieldValue(const VEC3F& pos) const {
        VEC3F v = pos;
        for (int i=0; i<iterations; i++) {
            v = this->field->getFieldValue(pos);
        }
        return v;
    }
};

class EscapingIteratedVF3D: public VectorField3D {
public:
    VectorField3D* field;
    unsigned iterations;
    Real escape;

    EscapingIteratedVF3D(VectorField3D* field, unsigned iterations, Real escape): field(field), iterations(iterations), escape(escape) {}

    virtual VEC3F getFieldValue(const VEC3F& pos) const {
        VEC3F v = pos;
        for (int i=0; i<iterations; i++) {
            VEC3F next = this->field->getFieldValue(v);
            if (next.norm() >= escape) {
                return v;
            }
            v = next;
        }
        return v;
    }
};

class NormalizedVF3D: public VectorField3D {
private:
    VectorField3D* field;
public:
    NormalizedVF3D(VectorField3D* field): field(field) {}

    virtual VEC3F getFieldValue(const VEC3F& pos) const {
        return this->field->getFieldValue(pos).normalized();
    }
};

class GradientField3D: public VectorField3D {
private:
    FieldFunction3D* field;
    Real eps;
public:
    GradientField3D(FieldFunction3D* field, Real eps): field(field), eps(eps) {}

    virtual VEC3F getFieldValue(const VEC3F& pos) const {
        return this->field->getNumericalGradient(pos, this->eps);
    }
};

class JacobianField3D: public MatrixField3D {
private:
    VectorField3D* field;
    Real eps;
public:
    JacobianField3D(VectorField3D* field, Real eps): field(field), eps(eps) {}

    virtual MAT3F getFieldValue(const VEC3F& pos) const {
        VEC3F gX = field->x->getNumericalGradient(pos, eps);
        VEC3F gY = field->y->getNumericalGradient(pos, eps);
        VEC3F gZ = field->z->getNumericalGradient(pos, eps);

        MAT3F m;
        m << gX[0], gX[1], gX[2],
             gY[0], gY[1], gY[2],
             gZ[0], gZ[1], gZ[2];

        return m.transpose();
    }
};

class GradientNormField3D: public FieldFunction3D {
private:
    FieldFunction3D* field;
    Real eps;
public:
    GradientNormField3D(FieldFunction3D* field, Real eps): field(field), eps(eps) {}

    virtual Real getFieldValue(const VEC3F& pos) const {
        return this->field->getNumericalGradient(pos, this->eps).norm();
    }
};

class ConstantFunction3D: public FieldFunction3D {
public:
    Real value;
    ConstantFunction3D( Real value ): value(value) {}

    virtual Real getFieldValue(const VEC3F& pos) const {
        (void) pos;
        return value;
    }

};

class Grid3D: public FieldFunction3D {
public:
    uint xRes, yRes, zRes;
    bool supportsNonIntegerIndices = false;

    // Bounds (aka center + lengths) for mapping into this grid
    // as a field function
    AABB mapBox;
    bool hasMapBox = false;

    virtual uint totalCells() const {
        return xRes * yRes * zRes;
    }

    virtual Real get(uint x, uint y, uint z) const = 0;

    virtual Real getf(Real x, Real y, Real z) const {
        (void) x; (void) y; (void) z; // Suppress unused argument warning
        printf("This grid doesn't support non-integer indices!\n");
        exit(1);
    }

    virtual Real get(VEC3I pos) const {
        return get(pos[0], pos[1], pos[2]);
    }

    virtual Real getf(VEC3F pos) const {
        return getf(pos[0], pos[1], pos[2]);
    }

    virtual void setMapBox(AABB box) {
        mapBox = box;
        hasMapBox = true;
    }

    virtual Real getFieldValue(const VEC3F& pos) const override {
        if (!hasMapBox) {
            printf("Attempting getFieldValue on a Grid3D without a mapBox!\n");
            exit(1);
        }

        VEC3F samplePoint = (pos - mapBox.min()).cwiseQuotient(mapBox.span());
        samplePoint = samplePoint.cwiseMax(VEC3F(0,0,0)).cwiseMin(VEC3F(1,1,1));

        const VEC3F indices = samplePoint.cwiseProduct(VEC3F(xRes-1, yRes-1, zRes-1));

        if (supportsNonIntegerIndices) {
            return getf(indices);
        } else {
            return get(indices.cast<int>());
        }
    }

    virtual VEC3F gridToFieldCoords(const VEC3F& pos) const {
        if (!hasMapBox) {
            printf("Attempting cellToFieldCoords on a Grid3D without a mapBox!\n");
            exit(1);
        }

        return mapBox.min() + pos.cwiseQuotient(VEC3F(xRes, yRes, zRes)).cwiseProduct(mapBox.span());
    }

    virtual VEC3F getCellCenter(const VEC3I& pos) const {
        if (!hasMapBox) {
            printf("Attempting getCellCenter on a Grid3D without a mapBox!\n");
            exit(1);
        }

        VEC3F cellCornerToCenter = mapBox.span().cwiseQuotient(VEC3F(xRes, yRes, zRes))/2.0;
        VEC3F posF(pos.x(), pos.y(), pos.z());

        return gridToFieldCoords(posF) + cellCornerToCenter;
    }

    virtual VEC3F getCellSize() const {
        if (!hasMapBox) {
            printf("Attempting getCellSize on a Grid3D without a mapBox!\n");
            exit(1);
        }

        return mapBox.span().cwiseQuotient( VEC3F(xRes, yRes, zRes) );
    }

    virtual void writeCSV(string filename, bool verbose = false) {
        ofstream out;
        out.open(filename);
        if (out.is_open() == false)
            return;

        PB_DECL();

        if (verbose) {
            PB_STARTD("Writing %d x %d x % d field (%d values) to %s", xRes, yRes, zRes, (xRes * yRes * zRes), filename.c_str());
        }

        for (uint i = 0; i < xRes; ++i) {
            for (uint j = 0; j < yRes; ++j) {
                for (uint k = 0; k < zRes; ++k) {
                    out << i << ", " << j << ", " << k << ", " << get(i, j, k) << endl;
                }
            }
            if (verbose) {
                PB_PROGRESS(((Real) i) / xRes);
            }

        }

        PB_END();

        out.close();
    }

    // Writes to VTK rectilinear grid format
    void writeVTK(string filename, bool verbose = false) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
            return;
        }

        PB_DECL();

        if (verbose) {
            PB_STARTD("Writing %d x %d x % d field (%d values) to %s", xRes, yRes, zRes, (xRes * yRes * zRes), filename.c_str());
        }

        // Write VTK header
        file << "# vtk DataFile Version 3.0\n";
        file << "Grid3D data\n";
        file << "ASCII\n";
        file << "DATASET RECTILINEAR_GRID\n";

        // Write dimensions
        file << "DIMENSIONS " << xRes << " " << yRes << " " << zRes << "\n";

        // Write coordinates (assuming uniform grid)
        file << "X_COORDINATES " << xRes << " float\n";
        for (uint i = 0; i < xRes; ++i) file << i << " ";
        file << "\n";

        file << "Y_COORDINATES " << yRes << " float\n";
        for (uint i = 0; i < yRes; ++i) file << i << " ";
        file << "\n";

        file << "Z_COORDINATES " << zRes << " float\n";
        for (uint i = 0; i < zRes; ++i) file << i << " ";
        file << "\n";

        // Write data
        file << "POINT_DATA " << totalCells() << "\n";
        file << "SCALARS value float 1\n";
        file << "LOOKUP_TABLE default\n";

        for (uint k = 0; k < zRes; ++k) {
            for (uint j = 0; j < yRes; ++j) {
                for (uint i = 0; i < xRes; ++i) {
                    Real val = get(i, j, k);
                    if (isinf(val) || isnan(val)) {
                        file << -123 << "\n";
                    } else {
                        file << val << "\n";
                    }
                }
            }
            if (verbose) {
                PB_PROGRESS(((Real) k) / zRes);
            }
        }

        if (verbose) {
            PB_END();
        }

        file.close();
    }

    // Writes to F3D file using the field bounds if the field has them, otherwise using
    // the resolution ofthe grid.
    void writeF3D(string filename, bool verbose = false) const {
        if (hasMapBox) {
            writeF3D(filename, mapBox, verbose);
        } else {
            writeF3D(filename, AABB(VEC3F(0,0,0), VEC3F(xRes, yRes, zRes)), verbose);
        }
    }

    void writeF3D(string filename, AABB bounds, bool verbose = false) const {
        FILE* file = fopen(filename.c_str(), "wb");

        if (file == NULL) {
            PRINT("Failed to write F3D: file open failed!");
            exit(0);
        }

        PB_DECL();
        if (verbose) {
            PB_STARTD("Writing %dx%dx%d field to %s", xRes, yRes, zRes, filename.c_str());
        }

        // write dimensions
        fwrite((void*)&xRes, sizeof(int), 1, file);
        fwrite((void*)&yRes, sizeof(int), 1, file);
        fwrite((void*)&zRes, sizeof(int), 1, file);

        MyEigen::write_vec3f(file, bounds.center());
        MyEigen::write_vec3f(file, bounds.span());

        const int totalCells = xRes*yRes*zRes;

        if (totalCells <= 0)
            return;

        // write data
        for (uint i = 0; i < xRes; ++i) {
            for (uint j = 0; j < yRes; ++j) {
                for (uint k = 0; k < zRes; ++k) {
                    const Real val = get(i, j, k);
                    double out;
                    if (sizeof(Real) != sizeof(double)) {
                        out = (double) val;
                    } else {
                        out = val;
                    }

                    fwrite((void*) (&out), sizeof(double), 1, file);

                }
            }

            if (verbose && i % 10 == 0) {
                PB_PROGRESS((Real) i / xRes);
            }
        }

        if (verbose) {
            PB_END();
        }
    }
};

class ArrayGrid3D: public Grid3D {
private:
    Real* values;
public:

    // Create empty (not zeroed) field with given resolution
    ArrayGrid3D(uint xRes, uint yRes, uint zRes) {
        this->xRes = xRes;
        this->yRes = yRes;
        this->zRes = zRes;
        values = new Real[xRes * yRes * zRes];
    }

    // Create empty (not zeroed) field with given resolution
    ArrayGrid3D(VEC3I resolution): ArrayGrid3D(resolution[0], resolution[1], resolution[2]) {}

    // Read ArrayGrid3D from F3D
    ArrayGrid3D(string filename, string format = "f3d", bool verbose = false) {

        if (format == "f3d") {
            FILE* file = fopen(filename.c_str(), "rb");
            if (file == NULL) {
                PRINT("Failed to read F3D: file open failed!");
                exit(0);
            }

            int xRes, yRes, zRes;
            VEC3F center, lengths;

            // read dimensions
            fread((void*)&xRes, sizeof(int), 1, file);
            fread((void*)&yRes, sizeof(int), 1, file);
            fread((void*)&zRes, sizeof(int), 1, file);

            MyEigen::read_vec3f(file, center);
            MyEigen::read_vec3f(file, lengths);

            this->xRes = xRes;
            this->yRes = yRes;
            this->zRes = zRes;

            try {
                values = new Real[xRes * yRes * zRes];
            } catch(bad_alloc& exc) {
                printf("Failed to allocate %.2f MB for ArrayGrid3D read from file!\n", (xRes * yRes * zRes * sizeof(Real)) / pow(2.0,20.0));
                exit(0);
            }

            if (verbose) {
                printf("Reading %d x %d x %d field from %s... ", xRes, yRes, zRes, filename.c_str());
                fflush(stdout);
            }

            AABB bounds((center - lengths/2), (center + lengths/2));
            setMapBox(bounds);

            const int totalCells = xRes * yRes * zRes;
            // always read in as a double
            if (sizeof(Real) != sizeof(double)) {
                double* dataDouble = new double[totalCells];
                fread((void*)dataDouble, sizeof(double), totalCells, file);

                for (int x = 0; x < totalCells; x++)
                    values[x] = dataDouble[x];

                if (verbose) printf("\n");

                delete[] dataDouble;
            } else fread((void*)values, sizeof(Real), totalCells, file);

            if (verbose) {
                printf("done.\n");
            }

        } else {
            PRINT("CSV import not implemented yet!");
            exit(1);
        }

    }
    // Destructor
    ~ArrayGrid3D() {
        delete values;
    }

    // Access value based on integer indices
    Real get(uint x, uint y, uint z) const override {
        return values[(z * yRes + y) * xRes + x];
    }

    // Access value directly (allows setting)
    Real& at(uint x, uint y, uint z) {
        return values[(z * yRes + y) * xRes + x];
    }

    Real& atFieldPos(VEC3F pos) {
        if (!hasMapBox) {
            printf("Attempting atFieldPos on an ArrayGrid without a mapBox!\n");
            exit(1);
        }

        VEC3F samplePoint = (pos - mapBox.min()).cwiseQuotient(mapBox.span());
        samplePoint = samplePoint.cwiseMax(VEC3F(0,0,0)).cwiseMin(VEC3F(1,1,1));

        const VEC3F indices = samplePoint.cwiseProduct(VEC3F(xRes-1, yRes-1, zRes-1));

        return at(indices[0], indices[1], indices[2]);
    }

    Real& atFieldPos(Real x, Real y, Real z) {
        return atFieldPos(VEC3F(x,y,z));
    }


    Real& operator()(VEC3F pos) {
        return atFieldPos(pos);
    }

    // Access value directly in C-style array (allows setting)
    Real& operator[](size_t x) {
        return values[x];
    }


    // Create field from scalar function by sampling it on a regular grid
    ArrayGrid3D(uint xRes, uint yRes, uint zRes, VEC3F functionMin, VEC3F functionMax, FieldFunction3D *fieldFunction):ArrayGrid3D(xRes, yRes, zRes){

        VEC3F gridResF(xRes, yRes, zRes);

        PB_START("Sampling %dx%dx%d scalar field into ArrayGrid3D", xRes, yRes, zRes);

        for (uint i = 0; i < xRes; i++) {
            for (uint j = 0; j < yRes; j++) {
                for (uint k = 0; k < zRes; k++) {
                    VEC3F gridPointF(i, j, k);
                    VEC3F fieldDelta = functionMax - functionMin;

                    VEC3F samplePoint = functionMin + (gridPointF.cwiseQuotient(gridResF - VEC3F(1,1,1)).cwiseProduct(fieldDelta));

                    Real val = fieldFunction->getFieldValue(samplePoint);

                    this->at(i, j, k) = val;
                }
            }
            PB_PROGRESS((Real) i / xRes);
        }
        PB_END();

        this->setMapBox(AABB(functionMin, functionMax));

    }


};

class VirtualGrid3D: public Grid3D {
private:
    FieldFunction3D *fieldFunction;
    VEC3F functionMin, functionMax;

public:

    VEC3F getSamplePoint(Real x, Real y, Real z) const {
        VEC3F gridPointF(x, y, z);
        VEC3F gridResF(xRes, yRes, zRes);
        VEC3F fieldDelta = functionMax - functionMin;
        // VEC3F samplePoint = functionMin + (gridPointF.cwiseQuotient(gridResF - VEC3F(1,1,1)).cwiseProduct(fieldDelta));
        VEC3F samplePoint = functionMin + (gridPointF.cwiseQuotient(gridResF).cwiseProduct(fieldDelta));

        return samplePoint;
    }

    VirtualGrid3D(uint xRes, uint yRes, uint zRes, VEC3F functionMin, VEC3F functionMax,  FieldFunction3D *fieldFunction):
        fieldFunction(fieldFunction),
        functionMin(functionMin),
        functionMax(functionMax) {
            this->xRes = xRes;
            this->yRes = yRes;
            this->zRes = zRes;

            this->setMapBox(AABB(functionMin, functionMax));

            this->supportsNonIntegerIndices = true;
        }

    // Virtually (shallow) resample an existing grid
    VirtualGrid3D(uint xRes, uint yRes, uint zRes, Grid3D *other):
        fieldFunction(other)
    {
        if (!other->hasMapBox) {
            PRINTF("Attempting to virtually resample a Grid3D without a mapBox!\n");
            exit(1);
        }

        this->functionMin = other->mapBox.min();
        this->functionMax = other->mapBox.max();
        this->setMapBox(other->mapBox);

        this->xRes = xRes;
        this->yRes = yRes;
        this->zRes = zRes;

        this->supportsNonIntegerIndices = true;

    }

    virtual Real get(uint x, uint y, uint z) const override {
        return getf(x, y, z);
    }

    virtual Real getf(Real x, Real y, Real z) const override {
        return fieldFunction->getFieldValue(getSamplePoint(x, y, z));
    }
};

// Hash function for Eigen matrix and vector.
// From https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template<typename T> struct matrix_hash {
    size_t operator()(T const& matrix) const {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (long i = 0; i < matrix.size(); ++i) { // For some reason Eigen size is not unsigned?!
            auto elem = *(matrix.data() + i);
            seed ^= hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};


class VirtualGrid3DCached: public VirtualGrid3D {
protected:
    mutable unordered_map<VEC3F, Real, matrix_hash<VEC3F>> map;

public:
    mutable int numQueries = 0;
    mutable int numHits = 0;
    mutable int numMisses = 0;

    using VirtualGrid3D::VirtualGrid3D;

    virtual Real get(uint x, uint y, uint z) const override {
        return getf(x,y,z);
    }

    virtual Real getf(Real x, Real y, Real z) const override {
        VEC3F key(x,y,z);
        numQueries++;

        auto search = map.find(key);
        if (search != map.end()) {
            numHits++;
            return search->second;
        }

        Real result = VirtualGrid3D::get(x,y,z);
        map[key] = result;
        numMisses++;
        return result;
    }

};

class VirtualGrid3DLimitedCache: public VirtualGrid3DCached {
private:
    size_t maxSize;
    mutable queue<VEC3F> cacheQueue;

public:
    // Instantiates a VirtualGrid3D with a limited-size cache. When additional
    // items are inserted into the cache (beyond the capacity), the cache will
    // forget the item that was least recently inserted. If capacity -1 is
    // specified (default), it defaults to a size equal to three XY slices
    // through the field, which is suited for marching cubes.
    VirtualGrid3DLimitedCache(uint xRes, uint yRes, uint zRes, VEC3F functionMin, VEC3F functionMax,  FieldFunction3D *fieldFunction, int capacity = -1):
        VirtualGrid3DCached(xRes, yRes, zRes, functionMin, functionMax, fieldFunction) {
            PRINTV3(functionMin);
            PRINTV3(functionMax);
            if (capacity == -1) {
                maxSize = xRes * yRes * 3;
            } else {
                maxSize = capacity;
            }
        }

    virtual Real get(uint x, uint y, uint z) const override {
        return getf(x,y,z);
    }

    virtual Real getf(Real x, Real y, Real z) const override {
        VEC3F key(x,y,z);
        numQueries++;

        auto search = map.find(key);
        if (search != map.end()) {
            numHits++;
            return search->second;
        }

        // We need to insert another value
        if (cacheQueue.size() >= maxSize) {
            map.erase(cacheQueue.front());
            cacheQueue.pop();
        }

        Real result = VirtualGrid3D::getf(x,y,z);
        map[key] = result;
        cacheQueue.push(key);

        numMisses++;
        return result;
    }
};


class InterpolationGrid: public Grid3D {
private:
    Real interpolate(Real x0, Real x1, Real d) const {
        switch (mode) {
        case LINEAR:
            return ((1 - d) * x0) + (d * x1);
        case SMOOTHSTEP:
            d = (3 * d * d) - (2 * d * d * d);
            return ((1 - d) * x0) + (d * x1);
        }
        assert(false);
        return -1;
    }


public:
    Grid3D* baseGrid;

    enum INTERPOLATION_MODE {
        LINEAR,
        SMOOTHSTEP
    };

    INTERPOLATION_MODE mode;

    InterpolationGrid(Grid3D* baseGrid, INTERPOLATION_MODE mode = LINEAR) {
        this->baseGrid = baseGrid;
        xRes = baseGrid->xRes;
        yRes = baseGrid->yRes;
        zRes = baseGrid->zRes;
        this->mode = mode;
        this->supportsNonIntegerIndices = true;

        if (baseGrid->hasMapBox) this->setMapBox(baseGrid->mapBox);

        if (baseGrid->supportsNonIntegerIndices) {
            PRINT("Warning: laying an InterpolationGrid over a grid which already supports non-integer indices!");
        }
    }

    virtual Real get(uint x, uint y, uint z) const override {
        return baseGrid->get(x, y, z);
    }

    virtual Real getf(Real x, Real y, Real z) const override {
        // "Trilinear" interpolation with whatever technique we select

        uint x0 = floor(x);
        uint y0 = floor(y);
        uint z0 = floor(z);

        uint x1 = x0 + 1;
        uint y1 = y0 + 1;
        uint z1 = z0 + 1;

        // Clamp if out of bounds
        x0 = (x0 > xRes - 1) ? xRes - 1 : x0;
        y0 = (y0 > yRes - 1) ? yRes - 1 : y0;
        z0 = (z0 > zRes - 1) ? zRes - 1 : z0;

        x1 = (x1 > xRes - 1) ? xRes - 1 : x1;
        y1 = (y1 > yRes - 1) ? yRes - 1 : y1;
        z1 = (z1 > zRes - 1) ? zRes - 1 : z1;


        const Real xd = min(1.0, max(0.0, (x - x0) / ((Real) x1 - x0)));
        const Real yd = min(1.0, max(0.0, (y - y0) / ((Real) y1 - y0)));
        const Real zd = min(1.0, max(0.0, (z - z0) / ((Real) z1 - z0)));

        // First grab 3D surroundings...
        const Real c000 = baseGrid->get(x0, y0, z0);
        const Real c001 = baseGrid->get(x0, y0, z1);
        const Real c010 = baseGrid->get(x0, y1, z0);
        const Real c011 = baseGrid->get(x0, y1, z1);
        const Real c100 = baseGrid->get(x1, y0, z0);
        const Real c101 = baseGrid->get(x1, y0, z1);
        const Real c110 = baseGrid->get(x1, y1, z0);
        const Real c111 = baseGrid->get(x1, y1, z1);

        // Now create 2D interpolated slice...
        const Real c00 = interpolate(c000, c100, xd);
        const Real c01 = interpolate(c001, c101, xd);
        const Real c10 = interpolate(c010, c110, xd);
        const Real c11 = interpolate(c011, c111, xd);

        // Extract 1D interpolated line...
        const Real c0 = interpolate(c00, c10, yd);
        const Real c1 = interpolate(c01, c11, yd);

        // And grab 0D point.
        const Real output = interpolate(c0, c1, zd);

        return output;
    }



};


class VectorGrid3D: public VectorField3D {
public:
    uint xRes, yRes, zRes;
    bool supportsNonIntegerIndices = false;

    // Bounds (aka center + lengths) for mapping into this grid
    // as a field function
    AABB mapBox;
    bool hasMapBox = false;

    virtual uint totalCells() const {
        return xRes * yRes * zRes;
    }

    virtual VEC3F get(uint x, uint y, uint z) const = 0;

    virtual VEC3F getf(Real x, Real y, Real z) const {
        (void) x; (void) y; (void) z; // Suppress unused argument warning
        printf("This grid doesn't support non-integer indices!\n");
        exit(1);
    }

    virtual VEC3F get(VEC3I pos) const {
        return get(pos[0], pos[1], pos[2]);
    }

    virtual VEC3F getf(VEC3F pos) const {
        return getf(pos[0], pos[1], pos[2]);
    }

    virtual void setMapBox(AABB box) {
        mapBox = box;
        hasMapBox = true;
    }

    virtual VEC3F getFieldValue(const VEC3F& pos) const override {
        if (!hasMapBox) {
            printf("Attempting getFieldValue on a VectorGrid3D without a mapBox!\n");
            exit(1);
        }

        VEC3F samplePoint = (pos - mapBox.min()).cwiseQuotient(mapBox.span());
        samplePoint = samplePoint.cwiseMax(VEC3F(0,0,0)).cwiseMin(VEC3F(1,1,1));

        const VEC3F indices = samplePoint.cwiseProduct(VEC3F(xRes-1, yRes-1, zRes-1));

        if (supportsNonIntegerIndices) {
            return getf(indices);
        } else {
            return get(indices.cast<int>());
        }
    }

    virtual VEC3F gridToFieldCoords(const VEC3F& pos) const {
        if (!hasMapBox) {
            printf("Attempting cellToFieldCoords on a Grid3D without a mapBox!\n");
            exit(1);
        }

        return mapBox.min() + pos.cwiseQuotient(VEC3F(xRes, yRes, zRes)).cwiseProduct(mapBox.span());
    }

    virtual VEC3F getCellCenter(const VEC3I& pos) const {
        if (!hasMapBox) {
            printf("Attempting getCellCenter on a Grid3D without a mapBox!\n");
            exit(1);
        }

        VEC3F cellCornerToCenter = mapBox.span().cwiseQuotient(VEC3F(xRes, yRes, zRes))/2.0;
        VEC3F posF(pos.x(), pos.y(), pos.z());

        return gridToFieldCoords(posF) + cellCornerToCenter;
    }

    // Writes to F3D file using the field bounds if the field has them, otherwise using
    // the resolution of the grid.
    virtual void writeF3Ds(string filename, bool verbose = false) const {
        if (hasMapBox) {
            writeF3Ds(filename, mapBox, verbose);
        } else {
            writeF3Ds(filename, AABB(VEC3F(0,0,0), VEC3F(xRes, yRes, zRes)), verbose);
        }
    }

    virtual void writeF3Ds(string filename, AABB bounds, bool verbose = false) const {
        VirtualGrid3D(xRes, yRes, zRes, bounds.min(), bounds.max(), this->x).writeF3D(filename + string(".x.f3d"), bounds, verbose);
        VirtualGrid3D(xRes, yRes, zRes, bounds.min(), bounds.max(), this->y).writeF3D(filename + string(".y.f3d"), bounds, verbose);
        VirtualGrid3D(xRes, yRes, zRes, bounds.min(), bounds.max(), this->z).writeF3D(filename + string(".z.f3d"), bounds, verbose);
    }

    virtual void writeCSV(string filename) {
        ofstream out;
        out.open(filename);
        if (out.is_open() == false)
            return;

        PB_START("Writing %d x %d x % d field (%d values) to %s", xRes, yRes, zRes, (xRes * yRes * zRes), filename.c_str());

        for (uint i = 0; i < xRes; ++i) {
            for (uint j = 0; j < yRes; ++j) {
                for (uint k = 0; k < zRes; ++k) {
                    VEC3F g = get(i, j, k);
                    out << i << ", " << j << ", " << k << ", " << g[0] << ", " << g[1] << ", " << g[2] << endl;
                }
            }
            PB_PROGRESS( ((Real) i) / xRes );
        }

        PB_END();

        out.close();

    }
};

class ArrayVectorGrid3D: public VectorGrid3D {
private:
    VEC3F* values;
public:

    // Create empty (not zeroed) field with given resolution
    ArrayVectorGrid3D(uint xRes, uint yRes, uint zRes) {
        this->xRes = xRes;
        this->yRes = yRes;
        this->zRes = zRes;
        values = new VEC3F[xRes * yRes * zRes];
    }

    // Create empty (not zeroed) field with given resolution
    ArrayVectorGrid3D(VEC3I resolution): ArrayVectorGrid3D(resolution[0], resolution[1], resolution[2]) {}

    // TODO Read ArrayGrid3D from F3Ds

    // Destructor
    ~ArrayVectorGrid3D() {
        delete values;
    }

    // Access value based on integer indices
    VEC3F get(uint x, uint y, uint z) const override {
        return values[(z * yRes + y) * xRes + x];
    }

    // Access value directly (allows setting)
    VEC3F& at(uint x, uint y, uint z) {
        return values[(z * yRes + y) * xRes + x];
    }

    VEC3F& atFieldPos(VEC3F pos) {
        if (!hasMapBox) {
            printf("Attempting atFieldPos on an ArrayGrid without a mapBox!\n");
            exit(1);
        }

        VEC3F samplePoint = (pos - mapBox.min()).cwiseQuotient(mapBox.span());
        samplePoint = samplePoint.cwiseMax(VEC3F(0,0,0)).cwiseMin(VEC3F(1,1,1));

        const VEC3F indices = samplePoint.cwiseProduct(VEC3F(xRes-1, yRes-1, zRes-1));

        return at(indices[0], indices[1], indices[2]);
    }

    VEC3F& atFieldPos(Real x, Real y, Real z) {
        return atFieldPos(VEC3F(x,y,z));
    }


    VEC3F& operator()(VEC3F pos) {
        return atFieldPos(pos);
    }

    // Access value directly in C-style array (allows setting)
    // VEC3F& operator[](size_t x) {
    //     return values[x];
    // }


    // Create field from scalar function by sampling it on a regular grid
    ArrayVectorGrid3D(uint xRes, uint yRes, uint zRes, VEC3F functionMin, VEC3F functionMax, VectorField3D *fieldFunction):ArrayVectorGrid3D(xRes, yRes, zRes){

        VEC3F gridResF(xRes, yRes, zRes);

        PB_START("Sampling %dx%dx%d vector field into ArrayVectorGrid3D...", xRes, yRes, zRes);

        for (uint i = 0; i < xRes; i++) {
            for (uint j = 0; j < yRes; j++) {
                for (uint k = 0; k < zRes; k++) {
                    VEC3F gridPointF(i, j, k);
                    VEC3F fieldDelta = functionMax - functionMin;

                    VEC3F samplePoint = functionMin + (gridPointF.cwiseQuotient(gridResF - VEC3F(1,1,1)).cwiseProduct(fieldDelta));

                    this->at(i, j, k) = fieldFunction->getFieldValue(samplePoint);
                }
            }
            PB_PROGRESS( ((Real) i)/xRes );
        }
        PB_END();

        this->setMapBox(AABB(functionMin, functionMax));

    }

};




#endif
