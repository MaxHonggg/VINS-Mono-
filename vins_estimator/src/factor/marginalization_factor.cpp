#include "marginalization_factor.h"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();//.data操作：返回一个直接指向内存中存储vector元素位置的指针
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);//对于先验因子，传入的参数块为 last_marginalization_blocks，该参数块经过了ceres优化

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    //ROS_WARN("release marginlizationinfo");
    
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete[] it->second;

    for (int i = 0; i < (int)factors.size(); i++)
    {

        delete[] factors[i]->raw_jacobians;
        
        delete factors[i]->cost_function;

        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);      //因子填充

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();//每一个优化变量块的 local_size，为ceres中自带函数

    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        double *addr = parameter_blocks[i];  //优化变量地址
        int size = parameter_block_sizes[i]; //优化变量长度
        parameter_block_size[reinterpret_cast<long>(addr)] = size;  //向成员变量parameter_block_size中填充：[所有优化变量地址] = 长度
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];  //待边缘化变量地址：由 drop_set 指定其位置
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;  //向parameter_block_idx中填充： [待边缘化变量地址] = 0 (后续将进行ID填充，ID按照变量块的长度递增)
    }
}

void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)
    {
        it->Evaluate();     //每个约束块进行残差项及雅克比矩阵计算更新

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes(); //每一个优化变量块的 local_size
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)  //遍历每一个变量块
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);//变量块地址
            int size = block_sizes[i];                                  //变量块长度
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                parameter_block_data[addr] = data;  //向parameter_block_data中填充： [所有优化变量地址] = 变量首地址
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const      //7 ==> 6 
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const     //6 ==> 7 
{
    return size == 6 ? 7 : size;
}

void* ThreadsConstructA(void* threadsstruct)    //构造大 Hessian 矩阵线程
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    for (auto it : p->sub_factors)//每一个约束块
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) //每个约束块中的每个优化变量块
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];//parameter_block_idx ：[所有变量地址] = ID
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            if (size_i == 7)
                size_i = 6;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;//对角线
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;//非对角线，对称
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for (auto &it : parameter_block_idx)    
    {
        it.second = pos;    //parameter_block_idx： 起始时，[待边缘化变量地址] = 0，从此处开始填充ID
        pos += localSize(parameter_block_size[it.first]);   //从parameter_block_idx中取地址，从parameter_block_size中取长度, [待边缘化变量] = ID + local_size
    }

    m = pos;        //待边缘化的变量总长度

    /*
        Q:为何要分两次存储边缘化变量与保留变量？？是否在存储空间上出现了先后顺序？？？
    */

    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())   //于此处填充parameter_block_idx,填充剩余的所有的变量信息 [剩余待保留变量地址] = ID+local_size 
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);            //累加所有变量的local_size
        }
    }

    n = pos - m;            //待保留的变量总长度

    //ROS_DEBUG("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());

    TicToc t_summing;
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();
    /*
    for (auto it : factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    ROS_INFO("summing up costs %f ms", t_summing.toc());
    */
    //multi thread


    TicToc t_thread_summing;
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }
    //-----------   四线程构建Hessian矩阵  ------------------------//
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);    //pos：所有变量的总local_size
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;   //parameter_block_size ：[所有优化变量地址] = 长度
        threadsstruct[i].parameter_block_idx = parameter_block_idx;     //parameter_block_idx ：  [待边缘化变量地址] = 0/ID，
        int ret = pthread_create( &tids[i], NULL, ThreadsConstructA ,(void*)&(threadsstruct[i]));//构造线程：任务函数ThreadsConstructA，输入数据threadsstruct[i]
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for( int i = NUM_THREADS - 1; i >= 0; i--)  
    {
        pthread_join( tids[i], NULL );          //线程启动，并行执行？？？
        A += threadsstruct[i].A;            //A矩阵带有了 变量顺序： 边缘化变量m维 | 非边缘化变量 n 维？？？？
        b += threadsstruct[i].b;
    }
    //------------------------------------------------------------------//

    //ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());

    //----------  构造线性化残差 与 线性化雅可比矩阵 用于先验约束块 ----------------------------------------------------------------------//
    //TODO
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());  //前m*m个一定是待边缘化的变量？？？
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);   //SelfAdjointEigenSolver：实对称矩阵可以保证对角化成功

    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * \
        Eigen::VectorXd(  (saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)   ).asDiagonal() * \//增加取逆数值稳定性，过小的特征值不再取逆
        /*
        conditon.select(DenseBase& thenMatrix,DenseBase& elseMatrix)函数：从元素中寻找满足condition的数，满足则赋值为thenMatrix，不满足则赋值为 elseMatrix
        示例：
        {
            MatrixXi m(3, 3);
            m << 1, 2, 3,
                4, 5, 6,
                7, 8, 9;
            m = (m.array() >= 5).select(-m, m);
            cout << m << endl;
        }
        输出为：
            1  2  3
            4 -5 -6
            -7 -8 -9
        */
                            saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    //如何从Hessian矩阵中恢复Jacobian矩阵与残差？？？？
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);   
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));    //将过小的特征值置为零
    //.array操作：Matrix expressions have an .array() method that 'converts' them into array expressions, so that coefficient-wise operations can be applied easily.Conversely, array expressions have a .matrix() method.  
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();     // Hessian矩阵特征值开方  J^T*J => J
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();// Hessian矩阵特征值取逆开方  J^T*J => J^T

    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();//恢复出 Jacobian 矩阵：               特征值开方*特征向量^T ？？
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;//恢复出残差 resudual=J^T*b：特征值取逆开方*特征向量^T *b, 已经带了负号？？？？？？？？？

    //---------  构造完成  -----------------------------------------------------------------------------------------------------------------//
    //std::cout << A << std::endl
    //          << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx) //parameter_block_idx为待边缘化的变量 地址~~local_size
    {
        if (it.second >= m) // m 为待边缘化的变量总的 local_size 大小
        {                   // 对于超过m的部分，全部为需要保留的变量，it.first 为变量地址===>param_block_idx中还存储了非边缘化变量？？？
            keep_block_size.push_back(parameter_block_size[it.first]);//需要保留的变量的local_size
            keep_block_idx.push_back(parameter_block_idx[it.first]);//需要保留的变量的id
            keep_block_data.push_back(parameter_block_data[it.first]);//需要保留的变量的地址

            //-----------------------------------------------------------------------------------------------//
            keep_block_addr.push_back(addr_shift[it.first]);//需要保留的变量的地址，下一轮转为了预积分因子的last_marginalization_parameter_blocks  ===> parameter_blocks，已经过本次优化？
            //-----------------------------------------------------------------------------------------------//

        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it); //mutable_parameter_block_sizes：不参与优化？？？？
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);     //设置残差数量 ： ？？？？？
};

//重写ceres的Evaluate函数，计算残差与jacobian
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);  //n：待保留优化变量个数
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;  //m：边缘化变量个数 ==>idx：将第一个保留变量的ID为0
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size); //将parameters[i]形状设置为size*1大小，从parameters[i]为首地址，连续取size个数据作为向量，
        //parameters 为传入的 last_marginalization_blocks ,
        //记录了该轮边缘化的所有变量x，这些变量已经由ceres进行了优化，将这些优化后的变量与上一轮边缘化的所有保留变量x0= keep_block_data作差 x-x0，得到变量的增量 dx ,再乘以雅克比矩阵，得到理论的残差

        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);//keep_block_data：需要保留的变量的地址，为上一轮边缘化后的保留变量
        if (size != 7)
            dx.segment(idx, size) = x - x0; //为何可以直接相减，变量已经对应好？？？？： keep_block_data[i] <====> last_marginalization_blocks[i]
        else
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }

    //-------先验残差的计算方法： rp - Hp * delta_x ---------------------------------------------//
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;//linearized_residuals 与 linearized_jacobians为边缘化后生成的， linearized_residuals已经带了负号？？？？？？？？？
    //---------------------------------------------------------------------------//
  
    //------- 构造雅可比矩阵  - --------------------------------------------//
    if (jacobians)
    {
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);

                int idx = marginalization_info->keep_block_idx[i] - m;
                
                //Map就是将原始“连续内存存储”的数据，以矩阵形式重新组织，减少数据复制开销
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);//jacobian(数据指针，行数，列数)
                //通过Map操作可以直接访问 jacobinas？？？

                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);     //即实现了对输入参数 jacobians的赋值？？？？
            }
        }
    }
    //---------------------------------------------------------------------------//

    return true;
}
