PLAT := 

CC := $(PLAT)gcc -std=c99
CPP := $(PLAT)g++ -std=c++11
LD := $(PLAT)ld
AR := $(PLAT)ar rcu 
RANLIB := $(PLAT)ranlib
STRIP := $(PLAT)strip
RM := rm -f

CFLAGS := 
CFLAGS += -O0 -g3 
CFLAGS += -Wall -DUSE_PC=0 -DUSE_MYMALLOC=1
#-Wstrict-prototypes 
CFLAGS += -I. -I../../flatbuffers/1.12.0/src/include -I../../kissfft/1.3.1/src -I../../gemmlowp/1.0.0/src -I../../../packs/ruy/1.0.0/src -I../../../src/test/lite/c -Itensorflow/lite/micro -I../../../src 

LDFLAGS += -lm 

OBJECTS := 

OBJECTS += tensorflow/lite/c/common.o

OBJECTS += tensorflow/lite/core/api/error_reporter.o
OBJECTS += tensorflow/lite/core/api/flatbuffer_conversions.o
OBJECTS += tensorflow/lite/core/api/op_resolver.o
OBJECTS += tensorflow/lite/core/api/tensor_utils.o

OBJECTS += ../../../src/micro_frontend/lib/fft.o
OBJECTS += ../../../src/micro_frontend/lib/fft_util.o
OBJECTS += ../../../src/micro_frontend/lib/filterbank.o
OBJECTS += ../../../src/micro_frontend/lib/filterbank_util.o
OBJECTS += ../../../src/micro_frontend/lib/frontend.o
OBJECTS += ../../../src/micro_frontend/lib/frontend_util.o
OBJECTS += ../../../src/micro_frontend/lib/log_lut.o
OBJECTS += ../../../src/micro_frontend/lib/log_scale.o
OBJECTS += ../../../src/micro_frontend/lib/log_scale_util.o
OBJECTS += ../../../src/micro_frontend/lib/noise_reduction.o
OBJECTS += ../../../src/micro_frontend/lib/noise_reduction_util.o
OBJECTS += ../../../src/micro_frontend/lib/pcan_gain_control.o
OBJECTS += ../../../src/micro_frontend/lib/pcan_gain_control_util.o
OBJECTS += ../../../src/micro_frontend/lib/window.o
OBJECTS += ../../../src/micro_frontend/lib/window_util.o

#mymalloc
OBJECTS += ../../../src/micro_frontend/lib/malloc.o

OBJECTS += tensorflow/lite/kernels/internal/quantization_util.o

OBJECTS += tensorflow/lite/kernels/kernel_util_.o
#new
OBJECTS += tensorflow/lite/micro/kernels/kernel_util.o

OBJECTS += tensorflow/lite/micro/kernels/activations.o
OBJECTS += tensorflow/lite/micro/kernels/add.o
OBJECTS += tensorflow/lite/micro/all_ops_resolver.o
OBJECTS += tensorflow/lite/micro/kernels/arg_min_max.o
OBJECTS += tensorflow/lite/micro/kernels/ceil.o
OBJECTS += tensorflow/lite/micro/kernels/comparisons.o
OBJECTS += tensorflow/lite/micro/kernels/concatenation.o
OBJECTS += tensorflow/lite/micro/kernels/conv.o
OBJECTS += tensorflow/lite/micro/kernels/depthwise_conv.o
OBJECTS += tensorflow/lite/micro/kernels/dequantize.o
OBJECTS += tensorflow/lite/micro/kernels/elementwise.o
OBJECTS += tensorflow/lite/micro/kernels/floor.o
OBJECTS += tensorflow/lite/micro/kernels/fully_connected.o
OBJECTS += tensorflow/lite/micro/kernels/logical.o
OBJECTS += tensorflow/lite/micro/kernels/logistic.o
OBJECTS += tensorflow/lite/micro/kernels/maximum_minimum.o
OBJECTS += tensorflow/lite/micro/kernels/mul.o
OBJECTS += tensorflow/lite/micro/kernels/neg.o
OBJECTS += tensorflow/lite/micro/kernels/pack.o
OBJECTS += tensorflow/lite/micro/kernels/pad.o
OBJECTS += tensorflow/lite/micro/kernels/pooling.o
OBJECTS += tensorflow/lite/micro/kernels/prelu.o
OBJECTS += tensorflow/lite/micro/kernels/quantize.o
OBJECTS += tensorflow/lite/micro/kernels/reshape.o
OBJECTS += tensorflow/lite/micro/kernels/round.o
OBJECTS += tensorflow/lite/micro/kernels/softmax.o
OBJECTS += tensorflow/lite/micro/kernels/split.o
OBJECTS += tensorflow/lite/micro/kernels/strided_slice.o
OBJECTS += tensorflow/lite/micro/kernels/svdf.o
OBJECTS += tensorflow/lite/micro/kernels/unpack.o


OBJECTS += tensorflow/lite/micro/memory_planner/greedy_memory_planner.o
OBJECTS += tensorflow/lite/micro/memory_planner/linear_memory_planner.o


OBJECTS += tensorflow/lite/micro/debug_log.o
#OBJECTS += tensorflow/lite/micro/debug_log_numbers.o
OBJECTS += tensorflow/lite/micro/memory_helpers.o
OBJECTS += tensorflow/lite/micro/micro_allocator.o
OBJECTS += tensorflow/lite/micro/micro_error_reporter.o
OBJECTS += tensorflow/lite/micro/micro_interpreter.o
#OBJECTS += tensorflow/lite/micro/micro_mutable_op_resolver.o
OBJECTS += tensorflow/lite/micro/micro_optional_debug_tools.o
OBJECTS += tensorflow/lite/micro/micro_utils.o
OBJECTS += tensorflow/lite/micro/simple_memory_allocator.o
OBJECTS += tensorflow/lite/micro/test_helpers.o

#OBJECTS += audio_provider_mock.o 
OBJECTS += ../../../src/audio_provider_test.o
#OBJECTS += yes_1000ms_sample_data.o 
#OBJECTS += no_1000ms_sample_data.o
OBJECTS += ../../../src/test/yes_30ms_sample_data.o
OBJECTS += ../../../src/test/yes_1000ms_sample_data.o
OBJECTS += ../../../src/test/no_30ms_sample_data.o
OBJECTS += ../../../src/test/no_1000ms_sample_data.o
#OBJECTS += ../../../src/audio_provider.o
OBJECTS += ../../../src/command_responder.o
OBJECTS += ../../../src/feature_provider.o
OBJECTS += ../../../src/micro_features/micro_features_generator.o
OBJECTS += ../../../src/micro_features/micro_model_settings.o
OBJECTS += ../../../src/micro_features/no_micro_features_data.o
OBJECTS += ../../../src/recognize_commands.o
#OBJECTS += ../../../src/tiny_conv_micro_features_model_data.o
OBJECTS += ../../../src/micro_features/yes_micro_features_data.o

OBJECTS += ../../../packs/kissfft/1.3.1/src/kiss_fft.o
OBJECTS += ../../../packs/kissfft/1.3.1/src/tools/kiss_fftr.o

#new
OBJECTS += ../../../src/main_functions.o
OBJECTS += ../../../src/micro_features/model.o
OBJECTS += tensorflow/lite/micro/micro_string.o

all : micro_speech

micro_speech: libtflite.a
	$(CPP) $(CFLAGS) ../../../src/main.cc libtflite.a -o $@ $(LDFLAGS)
	
libtflite.a : $(OBJECTS)
	$(AR) $@ $(OBJECTS) 
	$(RANLIB) $@

%.o : %.cc
	$(CPP) $(CFLAGS) -o $@ -c $<

%.o : %.cpp
	$(CPP) $(CFLAGS) -o $@ -c $<
	
%.o : %.c
	$(CC) $(CFLAGS) -o $@ -c $<
	
clean :
	$(RM) $(OBJECTS) *.a *.so micro_speech
