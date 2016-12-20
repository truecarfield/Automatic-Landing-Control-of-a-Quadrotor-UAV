#ifndef MATH_FIT
#define MATH_FIT
#include <vector>
/*

*/
namespace hector_quadrotor_controller{
    ///
    /// \brief This is the class for trajectory prediction
    ///
    class Fit{
        std::vector<double> factor; ///<The coefficients of the predicted trajectory
        double ssr;                 
        double sse;                 
        double rmse;                
        std::vector<double> fitedYs;
    public:
        Fit():ssr(0),sse(0),rmse(0){factor.resize(2,0);}
        ~Fit(){}

        void reset(){ssr = 0.0; sse = 0; rmse = 0; factor.resize(2,0); factor.clear();} //by ZD

        /// line fitting
        template<typename T>
        bool linearFit(const std::vector<T>& x, const std::vector<T>& y,bool isSaveFitYs=false)
        {
            return linearFit(&x[0],&y[0],getSeriesLength(x,y),isSaveFitYs);
        }
        template<typename T>
        bool linearFit(const T* x, const T* y,size_t length,bool isSaveFitYs=false)
        {
            factor.resize(2,0);
            T t1=0, t2=0, t3=0, t4=0;
            for(int i=0; i<length; ++i)
            {
                t1 += x[i]*x[i];
                t2 += x[i];
                t3 += x[i]*y[i];
                t4 += y[i];
            }
            factor[1] = (t3*length - t2*t4) / (t1*length - t2*t2);
            factor[0] = (t1*t4 - t2*t3) / (t1*length - t2*t2);
            //////////////////////////////////////////////////////////////////////////
            //error calculation
            calcError(x,y,length,this->ssr,this->sse,this->rmse,isSaveFitYs);
            return true;
        }
        ///
        /// \brief polynom fitting，y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n
        /// \param poly_n        The order of the polynom，e.g. if poly_n=2，then y=a0+a1*x+a2*x^2
        /// \param isSaveFitYs   Save the curve fitting results? 
        ///
        template<typename T>
        void polyfit(const std::vector<T>& x
            ,const std::vector<T>& y
            ,int poly_n
            ,bool isSaveFitYs=true)
        {
            polyfit(&x[0],&y[0],getSeriesLength(x,y),poly_n,isSaveFitYs);
        }
        template<typename T>
        void polyfit(const T* x,const T* y,size_t length,int poly_n,bool isSaveFitYs=true)
        {
            factor.resize(poly_n+1,0);
            int i,j;
            //double *tempx,*tempy,*sumxx,*sumxy,*ata;
            std::vector<double> tempx(length,1.0);

            std::vector<double> tempy(y,y+length);

            std::vector<double> sumxx(poly_n*2+1);
            std::vector<double> ata((poly_n+1)*(poly_n+1));
            std::vector<double> sumxy(poly_n+1);
            for (i=0;i<2*poly_n+1;i++){
                for (sumxx[i]=0,j=0;j<length;j++)
                {
                    sumxx[i]+=tempx[j];
                    tempx[j]*=x[j];
                }
            }
            for (i=0;i<poly_n+1;i++){
                for (sumxy[i]=0,j=0;j<length;j++)
                {
                    sumxy[i]+=tempy[j];
                    tempy[j]*=x[j];
                }
            }
            for (i=0;i<poly_n+1;i++)
                for (j=0;j<poly_n+1;j++)
                    ata[i*(poly_n+1)+j]=sumxx[i+j];
            gauss_solve(poly_n+1,ata,factor,sumxy);

            fitedYs.reserve(length);
            calcError(&x[0],&y[0],length,this->ssr,this->sse,this->rmse,isSaveFitYs);

        }
        ///
        ///
        void getFactor(std::vector<double>& factor){factor = this->factor;}
        ///
        ///
        void getFitedYs(std::vector<double>& fitedYs){fitedYs = this->fitedYs;}

        ///
        ///
        template<typename T>
        double getY(const T x) const
        {
            double ans(0);
            for (size_t i=0;i<factor.size();++i)
            {
                ans += factor[i]*pow((double)x,(int)i);
            }
            return ans;
        }
        ///
        ///
        double getSlope(){return factor[1];}
        ///
        ///
        double getIntercept(){return factor[0];}
        ///
        ///
        double getSSE(){return sse;}
        ///
        ///
        double getSSR(){return ssr;}
        ///
        ///
        double getRMSE(){return rmse;}
        ///
        ///
        double getR_square(){return 1-(sse/(ssr+sse));}
        ///
        ///
        template<typename T>
        size_t getSeriesLength(const std::vector<T>& x
            ,const std::vector<T>& y)
        {
            return (x.size() > y.size() ? y.size() : x.size());
        }
        ///
        template <typename T>
        static T Mean(const std::vector<T>& v)
        {
            return Mean(&v[0],v.size());
        }
        template <typename T>
        static T Mean(const T* v,size_t length)
        {
            T total(0);
            for (size_t i=0;i<length;++i)
            {
                total += v[i];
            }
            return (total / length);
        }
        ///
        ///
        size_t getFactorSize(){return factor.size();}
        ///
        ///
        double getFactor(size_t i){return factor.at(i);}
    private:
        template<typename T>
        void calcError(const T* x
            ,const T* y
            ,size_t length
            ,double& r_ssr
            ,double& r_sse
            ,double& r_rmse
            ,bool isSaveFitYs=true
            )
        {
            T mean_y = Mean<T>(y,length);
            T yi(0);
            fitedYs.reserve(length);
            for (int i=0; i<length; ++i)
            {
                yi = getY(x[i]);
                r_ssr += ((yi-mean_y)*(yi-mean_y));
                r_sse += ((yi-y[i])*(yi-y[i]));
                if (isSaveFitYs)
                {
                    fitedYs.push_back(double(yi));
                }
            }
            r_rmse = sqrt(r_sse/(double(length)));
        }
        template<typename T>
        void gauss_solve(int n
            ,std::vector<T>& A
            ,std::vector<T>& x
            ,std::vector<T>& b)
        {
            gauss_solve(n,&A[0],&x[0],&b[0]);
        }
        template<typename T>
        void gauss_solve(int n
            ,T* A
            ,T* x
            ,T* b)
        {
            int i,j,k,r;
            double max;
            for (k=0;k<n-1;k++)
            {
                max=fabs(A[k*n+k]); /*find maxmum*/
                r=k;
                for (i=k+1;i<n-1;i++){
                    if (max<fabs(A[i*n+i]))
                    {
                        max=fabs(A[i*n+i]);
                        r=i;
                    }
                }
                if (r!=k){
                    for (i=0;i<n;i++)         /*change array:A[k]&A[r] */
                    {
                        max=A[k*n+i];
                        A[k*n+i]=A[r*n+i];
                        A[r*n+i]=max;
                    }
                }
                max=b[k];                    /*change array:b[k]&b[r]     */
                b[k]=b[r];
                b[r]=max;
                for (i=k+1;i<n;i++)
                {
                    for (j=k+1;j<n;j++)
                        A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
                    b[i]-=A[i*n+k]*b[k]/A[k*n+k];
                }
            }

            for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
                for (j=i+1,x[i]=b[i];j<n;j++)
                    x[i]-=A[i*n+j]*x[j];
        }
    };
}


#endif
