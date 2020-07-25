

namespace TebPlanner
{
template<int D,typename E,typename V>
class TebUnaryEdge: public g2o::BaseUnaryEdge<D, E, V>
{
public:
    TebUnaryEdge()
    {
        _vertices[0] = NULL;
    }
    virtual ~TebUnaryEdge()
    {
        if(_vertices[0])
        {
            _vertices[0]->edges.erase(this);
        }
    }
    /**
     * @brief implement the virtual function
     */
    virtual bool read(std::istream& is)
    {
      return true;
    }

    /**
     * @brief implement the virtual function
     */
    virtual bool write(std::ostream& os) const
    {
      return os.good();
    }
    /**
     * @brief pass the configuration pointer to config_.
     * @param config: the configuration instance.
     */
    void setTebConfig(const Configuration& config)
    {
      config_ = &config;
    }


protected:
    Configuration* config_;//!<A pointer of configuration.


};
/**
 * @brief TebMultipleEdge class:Since the actual Edge class's resize() and setTebConfig() are the same,
 *                              use a base class to unify these functions.
 */
template<int D,typename E, typename V1,typename V2>
class TebBinaryEdge: public g2o::BaseBinaryEdge<D, E, V1, V2>
{
public:
    /**
     * @brief TebBinaryEdge:the default constructor.
     */
    TebBinaryEdge()
    {
        _vertices[0] = NULL;
        _vertices[1] = NULL;
    }
    /**
     * @brief ~TebBinaryEdge:destructor. The binaryEdge must have only two vertexs.
     */
    virtual ~TebBinaryEdge()
    {
        for (std::size_t i=0;i<2;++i)
        {
            if (_vertices[i])
            {
                _vertices[i]->edges.erase(this);
            }
        }
    }
    /**
     * @brief implement the virtual function
     */
    virtual bool read(std::istream& is)
    {
      return true;
    }

    /**
     * @brief implement the virtual function
     */
    virtual bool write(std::ostream& os) const
    {
      return os.good();
    }
    /**
     * @brief pass the configuration pointer to config_.
     * @param config: the configuration instance.
     */
    void setTebConfig(const Configuration& config)
    {
      config_ = &config;
    }


protected:
    Configuration* config_;//!<A pointer of configuration.

};
/**
 * @brief TebMultipleEdge class:Since the actual Edge class's resize() and setTebConfig() are the same,
 *                              use a base class to unify these functions.
 */
template<int D,typename E>
class TebMultipleEdge: public g2o::BaseMultiEdge<D,E>
{
public:
    /**
     * @brief TebMultipleEdge:the default constructor.
     */
    TebMultipleEdge(){}
    /**
     * @brief ~TebMultipleEdge:destructor.erase the vertex linked by the edge.
     */
    virtual ~TebMultipleEdge()
    {
        for (std::size_t i=0;i<_vertices.size();++i)
        {
            if (_vertices[i])
            {
                _vertices[i]->edges.erase(this);
            }
        }
    }
    /**
     * @brief resize:resize the vertexs' length
     * @param size: the size of vertexs
     */
    virtual void resize(size_t size)
    {
        g2o::BaseMultiEdge<D, E>::resize(size);
        for (std::size_t i=0;i<_vertices.size();++i)
        {
            _vertices[i] = NULL;
        }
    }
    /**
     * @brief implement the virtual function
     */
    virtual bool read(std::istream& is)
    {
      return true;
    }

    /**
     * @brief implement the virtual function
     */
    virtual bool write(std::ostream& os) const
    {
      return os.good();
    }
    /**
     * @brief pass the configuration pointer to config_.
     * @param config: the configuration instance.
     */
    void setTebConfig(const Configuration& config)
    {
      config_ = &config;
    }


protected:
    Configuration* config_;//!<A pointer of configuration.
};
}
