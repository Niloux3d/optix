#include "RayGeneratorClass.h"

// #include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

#include <cuda_runtime.h>

// #include <sutil/CUDAOutputBuffer.h>
#include <sutil/Exception.h>
#include <sutil/sutil.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string>

#include <sutil/Trackball.h>
#include <vector_types.h>
#include <sutil/vec_math.h>

#include <fstream>


template <typename T>
struct SbtRecord
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef SbtRecord<RayGenData>     RayGenSbtRecord;
typedef SbtRecord<MissData>       MissSbtRecord;
typedef SbtRecord<HitGroupData>   HitGroupSbtRecord;

const std::array<float, MATERIAL_TYPE_COUNT> g_material_reflections =
{ {
    0.5,    //unknown
    0.9,	//metal
    0.57,	//cement
    0.17,	//asphalt
    0.05,	//glass
    0.5,	//fabric
    0.7,	//skin
    0.6,	//plant
} };

static void context_log_cb( unsigned int level, const char* tag, const char* message, void* /*cbdata */)
{
    std::cerr << "RayGenerator::context_log_cb[" << std::setw( 2 ) << level << "][" << std::setw( 12 ) << tag << "]: "
              << message << "\n";
}

static float radians(float degrees)
{
    return degrees * M_PIf / 180.0f;
}

RayGenerator::RayGenerator()
{

    _context	            =	nullptr;
    _module	                =	nullptr;
    _raygen_prog_group	    =	nullptr;
    _miss_prog_group	    =	nullptr;
    _hitgroup_prog_group	=	nullptr;
    _pipeline	            =	nullptr;

	try
	{
		_param	                =	0;

		_gas_output_buffer   	=	0;
		_gas_output_buffer_size = 40 * 1024 * 1024;
		CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_gas_output_buffer ), _gas_output_buffer_size ) );

		_temp_buffer_gas	    =	0;
		_temp_buffer_gas_size = 30 * 1024 * 1024;
		CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_temp_buffer_gas ), _temp_buffer_gas_size ) );

		_vertices	            =	0;
		_vertices_size = 20 * 1024 * 1024;
		CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_vertices ), _vertices_size ) );

		_material_indices	    =	0;
		_material_indices_size  = 2 * 1024 * 1024;
		CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_material_indices ), _material_indices_size ) );

		CUDA_CHECK( cudaStreamCreate( &_stream ) );

	}
	catch( std::exception& e )
	{
	}

	memset(&m_f3Origin, 0x00, sizeof(float3));
	memset(&_sbt, 0x00, sizeof(OptixShaderBindingTable));

}

RayGenerator::~RayGenerator()
{
	try
	{
		if(_param != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _param) ) );
			_param = 0;
		}

		if(_gas_output_buffer != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _gas_output_buffer) ) );
			_gas_output_buffer = 0;
			_gas_output_buffer_size = 0;
		}

		if(_temp_buffer_gas != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _temp_buffer_gas) ) );
			_temp_buffer_gas = 0;
			_temp_buffer_gas_size = 0;
		}

		if(_vertices != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _vertices) ) );
			_vertices = 0;
			_vertices_size = 0;
		}

		if(_material_indices != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _material_indices) ) );
			_material_indices = 0;
			_material_indices_size = 0;
		}

		CUDA_CHECK( cudaStreamDestroy( _stream ) );
	}
	catch( std::exception& e )
	{
	}

}

bool RayGenerator::Initialize(SENSOR_TYPE t)
{
    try
    {
        //
        // Initialize CUDA and create OptiX context
        //
        {
            // Initialize CUDA
            CUDA_CHECK( cudaFree( 0 ) );

            // Initialize the OptiX API, loading all API entry points
            OPTIX_CHECK( optixInit() );

            // Specify context options
            OptixDeviceContextOptions options = {};
            options.logCallbackFunction       = &context_log_cb;
            options.logCallbackLevel          = 4;

            // Associate a CUDA context (and therefore a specific GPU) with this
            // device context
            CUcontext cuCtx = 0;  // zero means take the current context
            OPTIX_CHECK( optixDeviceContextCreate( cuCtx, &options, &_context ) );
        }

        //
        // Create module
        //
        OptixPipelineCompileOptions pipeline_compile_options = {};
        {
            OptixModuleCompileOptions module_compile_options = {};
            module_compile_options.maxRegisterCount     = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
            module_compile_options.optLevel             = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
            module_compile_options.debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO;

            pipeline_compile_options.usesMotionBlur        = false;
            pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
            //pipeline_compile_options.numPayloadValues      = 6;
            pipeline_compile_options.numPayloadValues      = 8;
            pipeline_compile_options.numAttributeValues    = 8; //TBD
#ifdef DEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
            pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
#else
            pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
#endif
            pipeline_compile_options.pipelineLaunchParamsVariableName = "params";
            pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

            std::string cuda_file_name;
            if(t == SENSOR_TYPE_RADAR){
                cuda_file_name = "RadarRayGenerator.ptx";
            }else{
                cuda_file_name = "RayGenerator.ptx";
            }

            // const std::string ptx = sutil::getPtxString("", 
            //                                             "/home/saimo/wuyou/project/optix/", 
            //                                             cuda_file_name.c_str());
            std::string ptx = readPTXFile("../common/RadarRayGenerator.ptx");
            // printf("ptx : %s\n", ptx.c_str());
            size_t sizeof_log = sizeof( log );

            OPTIX_CHECK_LOG( optixModuleCreateFromPTX(
                        _context,
                        &module_compile_options,
                        &pipeline_compile_options,
                        ptx.c_str(),
                        ptx.size(),
                        log,
                        &sizeof_log,
                        &_module
                        ) );
        }

        //
        // Create program groups
        //
        {
            OptixProgramGroupOptions program_group_options   = {}; // Initialize to zeros

            OptixProgramGroupDesc raygen_prog_group_desc    = {}; //
            raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
            raygen_prog_group_desc.raygen.module            = _module;
            raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__rg";
            size_t sizeof_log = sizeof( log );
            OPTIX_CHECK_LOG( optixProgramGroupCreate(
                        _context,
                        &raygen_prog_group_desc,
                        1,   // num program groups
                        &program_group_options,
                        log,
                        &sizeof_log,
                        &_raygen_prog_group
                        ) );

            OptixProgramGroupDesc miss_prog_group_desc  = {};
            miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
            miss_prog_group_desc.miss.module            = _module;
            miss_prog_group_desc.miss.entryFunctionName = "__miss__ms";
            sizeof_log = sizeof( log );
            OPTIX_CHECK_LOG( optixProgramGroupCreate(
                        _context,
                        &miss_prog_group_desc,
                        1,   // num program groups
                        &program_group_options,
                        log,
                        &sizeof_log,
                        &_miss_prog_group
                        ) );

            OptixProgramGroupDesc hitgroup_prog_group_desc = {};
            hitgroup_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
            hitgroup_prog_group_desc.hitgroup.moduleCH            = _module;
            hitgroup_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
            sizeof_log = sizeof( log );
            OPTIX_CHECK_LOG( optixProgramGroupCreate(
                        _context,
                        &hitgroup_prog_group_desc,
                        1,   // num program groups
                        &program_group_options,
                        log,
                        &sizeof_log,
                        &_hitgroup_prog_group
                        ) );
        }

        //
        // Link pipeline
        //
        {
            const uint32_t    max_trace_depth  = 1; //TBD
            OptixProgramGroup program_groups[] = { _raygen_prog_group, _miss_prog_group, _hitgroup_prog_group };

            OptixPipelineLinkOptions pipeline_link_options = {};
            pipeline_link_options.maxTraceDepth          = max_trace_depth;
            pipeline_link_options.debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
            size_t sizeof_log = sizeof( log );
            OPTIX_CHECK_LOG( optixPipelineCreate(
                        _context,
                        &pipeline_compile_options,
                        &pipeline_link_options,
                        program_groups,
                        sizeof( program_groups ) / sizeof( program_groups[0] ),
                        log,
                        &sizeof_log,
                        &_pipeline
                        ) );

            OptixStackSizes stack_sizes = {};
            for( auto& prog_group : program_groups )
            {
                OPTIX_CHECK( optixUtilAccumulateStackSizes( prog_group, &stack_sizes ) );
            }

            uint32_t direct_callable_stack_size_from_traversal;
            uint32_t direct_callable_stack_size_from_state;
            uint32_t continuation_stack_size;
            OPTIX_CHECK( optixUtilComputeStackSizes( &stack_sizes, max_trace_depth,
                                                     0,  // maxCCDepth. TBD
                                                     0,  // maxDCDEpth. TBD
                                                     &direct_callable_stack_size_from_traversal,
                                                     &direct_callable_stack_size_from_state, &continuation_stack_size ) );
            OPTIX_CHECK( optixPipelineSetStackSize( _pipeline, direct_callable_stack_size_from_traversal,
                                                    direct_callable_stack_size_from_state, continuation_stack_size,
                                                    1  // maxTraversableDepth. TBD
                                                    ) );
        }

        //
        // Set up shader binding table
        //
        {
            CUdeviceptr  raygen_record;
            const size_t raygen_record_size = sizeof( RayGenSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &raygen_record ), raygen_record_size ) );
            RayGenSbtRecord rg_sbt = {};
            OPTIX_CHECK( optixSbtRecordPackHeader( _raygen_prog_group, &rg_sbt ) );
            CUDA_CHECK( cudaMemcpy(
                        reinterpret_cast<void*>( raygen_record ),
                        &rg_sbt,
                        raygen_record_size,
                        cudaMemcpyHostToDevice
                        ) );

            CUdeviceptr miss_record;
            size_t      miss_record_size = sizeof( MissSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &miss_record ), miss_record_size ) );
            MissSbtRecord ms_sbt = {};
            OPTIX_CHECK( optixSbtRecordPackHeader( _miss_prog_group, &ms_sbt ) );
            CUDA_CHECK( cudaMemcpy(
                        reinterpret_cast<void*>( miss_record ),
                        &ms_sbt,
                        miss_record_size,
                        cudaMemcpyHostToDevice
                        ) );

            CUdeviceptr hitgroup_record;
            size_t      hitgroup_record_size = sizeof( HitGroupSbtRecord );
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &hitgroup_record ), hitgroup_record_size * MATERIAL_TYPE_COUNT ) );
            HitGroupSbtRecord hg_sbt[MATERIAL_TYPE_COUNT] = {};
            for( int i = 0; i < MATERIAL_TYPE_COUNT; ++i )
            {
                const int sbt_idx = i; // SBT for for ith material

                OPTIX_CHECK( optixSbtRecordPackHeader( _hitgroup_prog_group, &hg_sbt[sbt_idx] ) );
                hg_sbt[ sbt_idx ].data.reflection = g_material_reflections[i];
                printf("sbt_idx: %d, reflection: %f\n", sbt_idx, (hg_sbt[sbt_idx].data.reflection) );
            }

            CUDA_CHECK( cudaMemcpy(
                        reinterpret_cast<void*>( hitgroup_record ),
                        hg_sbt,
                        hitgroup_record_size * MATERIAL_TYPE_COUNT,
                        cudaMemcpyHostToDevice
                        ) );

            _sbt.raygenRecord                = raygen_record;
            _sbt.missRecordBase              = miss_record;
            _sbt.missRecordStrideInBytes     = sizeof( MissSbtRecord );
            _sbt.missRecordCount             = 1;
            _sbt.hitgroupRecordBase          = hitgroup_record;
            _sbt.hitgroupRecordStrideInBytes = sizeof( HitGroupSbtRecord );
            _sbt.hitgroupRecordCount         = MATERIAL_TYPE_COUNT;

        }
    }
    catch( std::exception& e )
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return false;
    }        
    return true;
}

bool RayGenerator::Uninitialize()
{

	if(_sbt.raygenRecord != 0)
	{
    	CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _sbt.raygenRecord) ) );
		_sbt.raygenRecord = 0;
	}

	if(_sbt.missRecordBase != 0)
	{
    	CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _sbt.missRecordBase) ) );
		_sbt.missRecordBase = 0;
	}

	if(_sbt.hitgroupRecordBase != 0)
	{
    	CUDA_CHECK( cudaFree( reinterpret_cast<void*>( _sbt.hitgroupRecordBase) ) );
		_sbt.hitgroupRecordBase = 0;
	}

	if(_pipeline != nullptr)
	{
		OPTIX_CHECK( optixPipelineDestroy( _pipeline ) );
	}

	if(_hitgroup_prog_group != nullptr)
	{
		OPTIX_CHECK( optixProgramGroupDestroy( _hitgroup_prog_group ) );
	}

	if(_miss_prog_group != nullptr)
	{
		OPTIX_CHECK( optixProgramGroupDestroy( _miss_prog_group ) );
	}

	if(_raygen_prog_group != nullptr)
	{
		OPTIX_CHECK( optixProgramGroupDestroy( _raygen_prog_group ) );
	}

	if(_module != nullptr)
	{
		OPTIX_CHECK( optixModuleDestroy( _module ) );
	}

	if(_context != nullptr)
	{
		OPTIX_CHECK( optixDeviceContextDestroy( _context ) );
	}


    return false;
}

void RayGenerator::setOrigin(const float3& o)
{
    m_f3Origin = o;
}

void RayGenerator::setEgoH(const float& h)
{
    m_fEgoH = h;
}
void RayGenerator::setEgoP(const float& p)
{
    m_fEgoP = p;
}
void RayGenerator::setEgoR(const float& r)
{
    m_fEgoR = r;
}

void RayGenerator::setScenarioData(ScenarioData& scenarioData)
{

    if((scenarioData.points.size() == 0) || (scenarioData.points.size() != scenarioData.materials.size() * 3))
    {
        return;
    }

    //
    // accel handling
    //
    {
        // Use default options for simplicity.  In a real use case we would want to
        // enable compaction, etc
        OptixAccelBuildOptions accel_options = {};
        accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS;
        accel_options.operation  = OPTIX_BUILD_OPERATION_BUILD;

        const size_t vertices_size = sizeof( float3 )*scenarioData.points.size();
		cuda_malloc(&_vertices, &_vertices_size, vertices_size);

        CUDA_CHECK( cudaMemcpy(
                    reinterpret_cast<void*>( _vertices ),
                    scenarioData.points.data(),
                    vertices_size,
                    cudaMemcpyHostToDevice
                    ) );

        const size_t mat_indices_size_in_bytes = scenarioData.materials.size() * sizeof( uint32_t );
		cuda_malloc(&_material_indices, &_material_indices_size, mat_indices_size_in_bytes);

		if(_material_indices != 0)
		{
			CUDA_CHECK( cudaMemcpy( reinterpret_cast<void*>( _material_indices ), scenarioData.materials.data(),
									mat_indices_size_in_bytes, cudaMemcpyHostToDevice ) );
		}
        //printf("_material_indices_size: %d \n", scenarioData.materials.size());

        // Our build input is a simple list of non-indexed triangle vertices
        const uint32_t triangle_input_flags[MATERIAL_TYPE_COUNT] = { 
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
            OPTIX_GEOMETRY_FLAG_NONE,
        };

        OptixBuildInput triangle_input = {};
        triangle_input.type                        = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
        triangle_input.triangleArray.vertexFormat  = OPTIX_VERTEX_FORMAT_FLOAT3;
		triangle_input.triangleArray.vertexStrideInBytes         = sizeof( float3 );
        triangle_input.triangleArray.numVertices   = static_cast<uint32_t>( scenarioData.points.size() );
        triangle_input.triangleArray.vertexBuffers = &_vertices;
        triangle_input.triangleArray.flags         = triangle_input_flags;
        triangle_input.triangleArray.numSbtRecords               = MATERIAL_TYPE_COUNT;
        triangle_input.triangleArray.sbtIndexOffsetBuffer        = _material_indices;
        triangle_input.triangleArray.sbtIndexOffsetSizeInBytes   = sizeof( uint32_t );
        triangle_input.triangleArray.sbtIndexOffsetStrideInBytes = sizeof( uint32_t );

        OptixAccelBufferSizes gas_buffer_sizes;
        OPTIX_CHECK( optixAccelComputeMemoryUsage(
                    _context,
                    &accel_options,
                    &triangle_input,
                    1, // Number of build inputs
                    &gas_buffer_sizes
                    ) );
		cuda_malloc(&_temp_buffer_gas, &_temp_buffer_gas_size, gas_buffer_sizes.tempSizeInBytes);

		cuda_malloc(&_gas_output_buffer, &_gas_output_buffer_size, gas_buffer_sizes.outputSizeInBytes);

        OPTIX_CHECK( optixAccelBuild(
                    _context,
                    0,                  // CUDA stream
                    &accel_options,
                    &triangle_input,
                    1,                  // num build inputs
                    _temp_buffer_gas,
                    gas_buffer_sizes.tempSizeInBytes,
                    _gas_output_buffer,
                    gas_buffer_sizes.outputSizeInBytes,
                    &_gas_handle,
                    nullptr,            // emitted property list
                    0                   // num emitted properties
                    ) );
    }

}

void RayGenerator::runOptix(Params params,
                            unsigned int width,
                            unsigned int height,
                            unsigned int depth)
{

    try
    {
        if (_param == 0)
        {
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_param ), sizeof( Params ) ) );
        }
            CUDA_CHECK(cudaMemcpy(
            reinterpret_cast<void *>(_param),
            &params, sizeof(params),
            cudaMemcpyHostToDevice));
            OPTIX_CHECK(optixLaunch(_pipeline, _stream, _param, sizeof(Params), &_sbt, width, height, depth));
            CUDA_SYNC_CHECK();
    }
    catch (std::exception &e)
    {
        return;
    }

    return;
}

void RayGenerator::runOptixRadar(RadarParams params,
                            unsigned int width,
                            unsigned int height,
                            unsigned int depth)
{

    try
    {
        if (_param == 0)
        {
            CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &_param ), sizeof( RadarParams ) ) );
        }
        CUDA_CHECK(cudaMemcpy(
            reinterpret_cast<void *>(_param),
            &params, sizeof(params),
            cudaMemcpyHostToDevice));
        OPTIX_CHECK(optixLaunch(_pipeline, _stream, _param, sizeof(RadarParams), &_sbt, width, height, depth));
        CUDA_SYNC_CHECK();
    }
    catch (std::exception &e)
    {
        return;
    }

    return;
}

void RayGenerator::cuda_malloc(CUdeviceptr* buf, size_t* size, size_t new_size)
{

	if((buf == NULL) || (size == NULL))
	{
		return;
	}

	if(*size < new_size)
	{
		if(*buf != 0)
		{
			CUDA_CHECK( cudaFree( reinterpret_cast<void*>( *buf) ) );
		}
		*buf = 0;

		CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( buf ), new_size ) );
		*size = new_size;
	}


}

std::string RayGenerator::readPTXFile(const char* filename)
{
    std::ifstream file(filename, std::ios::in | std::ios::binary | std::ios::ate);

    if (file.is_open()) {
        std::streampos fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> fileContent(fileSize);
        file.read(fileContent.data(), fileSize);

        file.close();

        return std::string(fileContent.data(), fileSize);
    }

    return "";
}