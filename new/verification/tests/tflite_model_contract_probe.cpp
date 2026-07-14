#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "tensorflow/lite/schema/schema_generated.h"

namespace {

void PrintTensor(const tflite::Tensor& tensor, const char* role) {
    std::cout << role << " name=" << (tensor.name() == nullptr ? "" : tensor.name()->c_str())
              << " type=" << tflite::EnumNameTensorType(tensor.type())
              << " shape=";
    if (tensor.shape() != nullptr) {
        for (flatbuffers::uoffset_t i = 0; i < tensor.shape()->size(); ++i) {
            if (i != 0) std::cout << 'x';
            std::cout << tensor.shape()->Get(i);
        }
    }
    const auto* quantization = tensor.quantization();
    if (quantization != nullptr && quantization->scale() != nullptr &&
        quantization->scale()->size() == 1 && quantization->zero_point() != nullptr &&
        quantization->zero_point()->size() == 1) {
        std::cout << " scale=" << quantization->scale()->Get(0)
                  << " zero_point=" << quantization->zero_point()->Get(0);
    }
    std::cout << '\n';
}

bool TensorHasContract(const tflite::Tensor& tensor,
                       tflite::TensorType type,
                       const std::initializer_list<int>& shape,
                       float scale,
                       std::int64_t zero_point) {
    if (tensor.type() != type || tensor.shape() == nullptr ||
        tensor.shape()->size() != shape.size()) {
        return false;
    }
    std::size_t index = 0;
    for (const int expected : shape) {
        if (tensor.shape()->Get(index++) != expected) return false;
    }
    const auto* quantization = tensor.quantization();
    return quantization != nullptr && quantization->scale() != nullptr &&
           quantization->scale()->size() == 1 && quantization->zero_point() != nullptr &&
           quantization->zero_point()->size() == 1 &&
           std::fabs(quantization->scale()->Get(0) - scale) <= 1.0e-5F &&
           quantization->zero_point()->Get(0) == zero_point;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " MODEL.tflite\n";
        return 2;
    }
    std::ifstream input(argv[1], std::ios::binary);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),
                                    std::istreambuf_iterator<char>());
    if (bytes.empty()) {
        std::cerr << "empty model\n";
        return 3;
    }
    const tflite::Model* model = tflite::GetModel(bytes.data());
    if (model == nullptr || model->subgraphs() == nullptr ||
        model->subgraphs()->size() != 1) {
        std::cerr << "invalid model/subgraph contract\n";
        return 4;
    }
    const tflite::SubGraph* graph = model->subgraphs()->Get(0);
    if (graph->tensors() == nullptr || graph->inputs() == nullptr ||
        graph->outputs() == nullptr) {
        std::cerr << "missing tensor contract\n";
        return 5;
    }
    std::cout << "schema_version=" << model->version()
              << " inputs=" << graph->inputs()->size()
              << " outputs=" << graph->outputs()->size() << '\n';
    for (flatbuffers::uoffset_t i = 0; i < graph->inputs()->size(); ++i) {
        PrintTensor(*graph->tensors()->Get(graph->inputs()->Get(i)), "input");
    }
    for (flatbuffers::uoffset_t i = 0; i < graph->outputs()->size(); ++i) {
        PrintTensor(*graph->tensors()->Get(graph->outputs()->Get(i)), "output");
    }
    if (model->operator_codes() != nullptr) {
        for (flatbuffers::uoffset_t i = 0; i < model->operator_codes()->size(); ++i) {
            const auto* code = model->operator_codes()->Get(i);
            std::cout << "op index=" << i
                      << " builtin=" << tflite::EnumNameBuiltinOperator(
                             code->builtin_code()) << '\n';
        }
    }
    if (model->version() != 3 || graph->inputs()->size() != 1 ||
        graph->outputs()->size() != 1 ||
        !TensorHasContract(*graph->tensors()->Get(graph->inputs()->Get(0)),
                           tflite::TensorType_INT8, {1, 32, 32, 1}, 1.0F / 255.0F, -128) ||
        !TensorHasContract(*graph->tensors()->Get(graph->outputs()->Get(0)),
                           tflite::TensorType_INT8, {1, 4}, 0.148821F, 19)) {
        std::cerr << "input/output tensor contract mismatch\n";
        return 6;
    }
    constexpr std::array<tflite::BuiltinOperator, 5> kExpectedOps = {
        tflite::BuiltinOperator_SPACE_TO_DEPTH,
        tflite::BuiltinOperator_CONV_2D,
        tflite::BuiltinOperator_MAX_POOL_2D,
        tflite::BuiltinOperator_MEAN,
        tflite::BuiltinOperator_FULLY_CONNECTED,
    };
    if (model->operator_codes() == nullptr ||
        model->operator_codes()->size() != kExpectedOps.size()) {
        std::cerr << "operator count contract mismatch\n";
        return 7;
    }
    for (std::size_t i = 0; i < kExpectedOps.size(); ++i) {
        if (model->operator_codes()->Get(i)->builtin_code() != kExpectedOps[i]) {
            std::cerr << "operator contract mismatch at index " << i << '\n';
            return 8;
        }
    }
    return 0;
}
